//! `--mode real` 라켓 헤드·리니어 레일 제어 진입점 — 조립 · 메인 루프 · 요약.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, ensure};
use crossbeam_channel::{Receiver, bounded, unbounded};
use pingpong_bot::camera::{Calibration, CamCliArgs, CamStreamArgs, StereoOfflineArgs};
use pingpong_bot::defaults::{
    self, DEFAULT_STEREO_CAM_ROLES, camera_params_for, detector_for, robot,
};
use pingpong_bot::hardware::RealHardware;
use pingpong_bot::hardware::dynamixel::DynamixelConfig;
use pingpong_bot::hardware::rail::RailConfig;
use pingpong_bot::robot::motion::InterceptWindow;
use tracing::{info, warn};

use crate::cli::Args;

use super::camera_worker::{self, CameraStats};
use super::estimator_worker::{self, EstimatorStats};
use super::fmt::{f2, f2_slice};
use super::{
    Options, PacedSource, PreviewEvent, PreviewWindow, RuntimeEvent, ShutdownGuard, control_worker,
    shutdown_channel, sim_host,
};

/// 카메라 → 추정 버퍼. 실시간이라 크게 잡을 이유가 없다 (밀리면 어차피 버린다).
const VISION_CAPACITY: usize = 8;
const PREVIEW_CAPACITY: usize = 2;
const SIM_CAPACITY: usize = 2;
/// 프리뷰가 없을 때 메인 루프 tick.
const IDLE_TICK: Duration = Duration::from_millis(5);

/// 라켓 헤드 x를 추정한 공 x에 맞추고 상대편 끝선 중앙을 조준한다.
/// 종료는 ESC·`q`(preview) 또는 제어 워커 `Done`이다.
pub fn run(args: &Args) -> Result<()> {
    let options = Options::from_args(args);
    let robot = robot().context("defaults::robot")?;
    let arm = Arc::clone(&robot.arm);

    let mut hardware = open_hardware(&options)?;
    // 카메라·추정·제어 스레드를 시작하기 전에 실기 자세부터 확정한다. 초기화 중에
    // 공을 잘못 추적하거나, 정렬 전 포즈를 sim에 보내는 일을 막는다.
    if options.home {
        info!("시작 자세 초기화 — 레일·관절을 준비 자세로 이동");
        let pose = control_worker::initialize_pose(&mut hardware, &arm)
            .map_err(|error| anyhow::anyhow!("시작 자세 초기화 실패: {error}"))?;
        info!(
            rail_x = f2(pose.rail_x),
            joints = %f2_slice(&pose.joints.values),
            "시작 자세 초기화 완료"
        );
    }
    let calibration = load_calibration()?;
    let sources = open_cameras(&options)?;
    ensure!(
        sources.len() >= calibration.min_cameras_for_triangulation(),
        "삼각측량에 카메라 {}대가 필요한데 {}대만 열렸다",
        calibration.min_cameras_for_triangulation(),
        sources.len()
    );

    let (guard, shutdown) = shutdown_channel();
    let (vision_tx, vision_rx) = bounded(VISION_CAPACITY);
    let vision_evict_rx = vision_rx.clone();
    // 제어 워커가 계획 중일 때는 이전 요청을 버리고 최신 한 건만 남긴다.
    let (commit_tx, commit_rx) = bounded(1);
    let commit_evict_rx = commit_rx.clone();
    let (event_tx, event_rx) = unbounded();
    let (preview_tx, preview_rx) = if options.preview {
        let (tx, rx) = bounded(PREVIEW_CAPACITY);
        (Some(tx), Some(rx))
    } else {
        (None, None)
    };
    let (sim_tx, sim_handle) = if options.sim {
        let (tx, rx) = bounded(SIM_CAPACITY);
        match sim_host::spawn(rx) {
            Some(handle) => (Some(tx), Some(handle)),
            None => (None, None),
        }
    } else {
        (None, None)
    };

    let mut camera_handles: Vec<JoinHandle<CameraStats>> = Vec::with_capacity(sources.len());
    for (resolved, source) in sources {
        let camera_id = resolved.camera_id;
        let detector =
            detector_for(camera_id).with_context(|| format!("detector_for cam{}", camera_id.0))?;
        let params = camera_params_for(camera_id)
            .with_context(|| format!("camera_params_for cam{}", camera_id.0))?;
        camera_handles.push(camera_worker::spawn(
            source,
            Box::new(detector),
            params,
            vision_tx.clone(),
            vision_evict_rx.clone(),
            shutdown.clone(),
        ));
    }
    // 원본 sender를 놓아야 카메라가 모두 끝났을 때 추정 워커가 Disconnected를 본다.
    drop(vision_tx);

    let estimator_handle = estimator_worker::spawn(
        vision_rx,
        calibration,
        InterceptWindow::default(),
        commit_tx,
        commit_evict_rx,
        preview_tx,
        sim_tx.clone(),
        event_tx.clone(),
        shutdown.clone(),
    );
    let control_handle = control_worker::spawn(
        Box::new(hardware),
        Arc::clone(&arm),
        commit_rx,
        sim_tx,
        event_tx,
        shutdown,
    );

    let outcome = main_loop(&options, &event_rx, preview_rx, guard);

    let camera_stats: Vec<CameraStats> = camera_handles
        .drain(..)
        .filter_map(|handle| handle.join().ok())
        .collect();
    let estimator_stats = estimator_handle.join().ok();
    if control_handle.join().is_err() {
        warn!("제어 워커 패닉");
    }
    if let Some(handle) = sim_handle {
        let _ = handle.join();
    }

    log_summary(&outcome, &camera_stats, estimator_stats.as_ref());
    return Ok(());
}

/// 세션 요약용 — 추적한 공과 보낸 제어 명령 수.
struct Outcome {
    tracks_seen: u64,
    commands_sent: u64,
    last: LastState,
}

enum LastState {
    None,
    Commanded,
    Failed(String),
    TimedOut,
    Quit,
}

impl Outcome {
    fn label(&self) -> String {
        let last = match &self.last {
            LastState::None => "없음".to_owned(),
            LastState::Commanded => "레일·라켓 조준 명령".to_owned(),
            LastState::Failed(reason) => format!("실패 - {reason}"),
            LastState::TimedOut => "타임아웃 - 공이 오지 않음".to_owned(),
            LastState::Quit => "사용자 종료".to_owned(),
        };
        return format!(
            "tracks={} commands={} last={last}",
            self.tracks_seen, self.commands_sent
        );
    }
}

/// 런타임 이벤트를 찍고 프리뷰를 돌린다.
fn main_loop(
    options: &Options,
    event_rx: &Receiver<RuntimeEvent>,
    preview_rx: Option<Receiver<PreviewEvent>>,
    guard: ShutdownGuard,
) -> Outcome {
    let mut preview = options.preview.then(|| PreviewWindow::new("real shot"));
    let mut guard = Some(guard);
    let mut wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
    let mut outcome = Outcome {
        tracks_seen: 0,
        commands_sent: 0,
        last: LastState::None,
    };
    let mut timed_out_warned = false;

    let result = loop {
        let mut control_done = false;
        while let Ok(event) = event_rx.try_recv() {
            log_event(&event);
            if let Some(preview) = &mut preview
                && let Some(lines) = result_lines(&event)
            {
                preview.set_result(lines);
            }
            match &event {
                RuntimeEvent::Ready { .. } => {
                    wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
                    timed_out_warned = false;
                }
                RuntimeEvent::Tracking { track_seq, .. } => {
                    outcome.tracks_seen = outcome.tracks_seen.max(*track_seq);
                    wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
                    timed_out_warned = false;
                }
                RuntimeEvent::Commanded { .. } => {
                    outcome.commands_sent += 1;
                    outcome.last = LastState::Commanded;
                }
                RuntimeEvent::Failed { reason, .. } => {
                    outcome.last = LastState::Failed(reason.clone());
                }
                RuntimeEvent::Done => control_done = true,
            }
        }

        if control_done {
            if !options.preview {
                break outcome;
            }
            // preview: Done이어도 창이 있으면 ESC까지 화면 유지. 워커는 이미 끝.
        }

        if !timed_out_warned && Instant::now() >= wait_deadline {
            warn!(
                timeout_secs = f2(options.timeout_secs),
                "공을 기다리다 시간 초과 — 세션은 유지"
            );
            outcome.last = LastState::TimedOut;
            timed_out_warned = true;
        }

        match &mut preview {
            Some(preview) => {
                if let Some(rx) = &preview_rx {
                    while let Ok(event) = rx.try_recv() {
                        preview.push(event);
                    }
                }
                if preview.show() {
                    outcome.last = LastState::Quit;
                    break outcome;
                }
            }
            None => {
                if control_done {
                    break outcome;
                }
                thread::sleep(IDLE_TICK);
            }
        }
    };

    drop(guard.take());
    if let Some(preview) = &preview {
        preview.close();
    }
    return result;
}

/// 최근 제어 결과 HUD (ASCII — Hershey 폰트 제약).
fn result_lines(event: &RuntimeEvent) -> Option<Vec<String>> {
    return match event {
        RuntimeEvent::Commanded {
            track_seq,
            stage,
            target,
            rail_x,
            aim_rad,
        } => Some(vec![
            format!("COMMAND track {track_seq} {stage:?}"),
            format!(
                "target x{} y{} z{}",
                f2(target.coords.x),
                f2(target.coords.y),
                f2(target.coords.z)
            ),
            format!("rail {}  aim {} deg", f2(*rail_x), f2(aim_rad.to_degrees())),
        ]),
        RuntimeEvent::Failed { track_seq, reason } => {
            Some(vec![format!("FAILED track {track_seq:?}"), reason.clone()])
        }
        _ => None,
    };
}

fn open_hardware(options: &Options) -> Result<RealHardware> {
    let mut dxl = DynamixelConfig::default();
    if let Some(port) = &options.dxl_port {
        dxl.port = port.clone();
    }
    dxl.hold_torque_on_close = !options.release_torque;
    let rail = RailConfig::default();
    info!(
        port = %dxl.port,
        dry_run = options.dry_run,
        rail_enabled = rail.enabled,
        hold_torque_on_close = dxl.hold_torque_on_close,
        "real 하드웨어 (mirror ID1↔ID2)"
    );
    let hardware = if options.dry_run {
        RealHardware::dry_run(dxl, Some(rail))
    } else {
        RealHardware::new(dxl, Some(rail))
    };
    return hardware.context("하드웨어 초기화");
}

fn load_calibration() -> Result<Calibration> {
    let path = defaults::calibration_path();
    let calibration = Calibration::load_json(&path)
        .map_err(anyhow::Error::msg)
        .with_context(|| format!("calibration 로드: {}", path.display()))?;
    info!(
        cameras = calibration.camera_count(),
        path = %path.display(),
        "calibration"
    );
    return Ok(calibration);
}

type OpenedCameras = Vec<(
    pingpong_bot::camera::ResolvedCam,
    Box<dyn pingpong_bot::camera::FrameSource>,
)>;

/// 라이브 캠, 또는 `--clip`이면 녹화 클립.
///
/// 클립은 **녹화 당시 fps로 페이싱**해서 재생한다 ([`PacedSource`]) — 그래야 계획 스로틀·
/// 요청 신선도·하드웨어 `stream_hz` 같은 벽시계 로직이 라이브와 같은 조건에서 돈다.
fn open_cameras(options: &Options) -> Result<OpenedCameras> {
    let cams = CamCliArgs {
        cam: DEFAULT_STEREO_CAM_ROLES.to_vec(),
        stream: CamStreamArgs::default(),
    };
    let Some(clip) = &options.clip else {
        return cams
            .open_sources()
            .map_err(anyhow::Error::msg)
            .context("실캠 열기");
    };

    let offline = StereoOfflineArgs {
        clip: Some(clip.clone()),
    };
    let resolved = offline
        .resolve()
        .map_err(anyhow::Error::msg)
        .context("클립 해석")?
        .context("클립을 찾지 못했다")?;
    resolved.log();
    info!(
        dir = %resolved.dir.display(),
        meas_fps = resolved.meas_fps.map(f2),
        "클립 재생 — 라이브 캠 대신"
    );

    // 파일 소스는 `--cam` 역할 순서대로 camera::Id를 받는다 (left → Id(0), right → Id(1)).
    let sources = cams
        .open_file_sources(&resolved.paths(), resolved.meas_fps)
        .map_err(anyhow::Error::msg)
        .context("클립 열기")?;
    let resolved_cams = cams.resolve().map_err(anyhow::Error::msg)?;
    return Ok(resolved_cams
        .into_iter()
        .zip(sources)
        .map(|(cam, source)| {
            let paced: Box<dyn pingpong_bot::camera::FrameSource> =
                Box::new(PacedSource::new(source));
            (cam, paced)
        })
        .collect());
}

fn log_event(event: &RuntimeEvent) {
    match event {
        RuntimeEvent::Ready { pose } => info!(
            rail_x = f2(pose.rail_x),
            joints = f2_slice(&pose.joints.values),
            "실기 단순 제어 준비"
        ),
        RuntimeEvent::Tracking {
            track_seq,
            position,
            speed,
        } => info!(
            track = track_seq,
            x = f2(position.coords.x),
            y = f2(position.coords.y),
            z = f2(position.coords.z),
            speed = f2(*speed),
            "공 궤적 추적 시작"
        ),
        RuntimeEvent::Commanded {
            track_seq,
            stage,
            target,
            rail_x,
            aim_rad,
        } => info!(
            track = track_seq,
            ?stage,
            target_x = f2(target.coords.x),
            target_y = f2(target.coords.y),
            target_z = f2(target.coords.z),
            rail_x = f2(*rail_x),
            aim_deg = f2(aim_rad.to_degrees()),
            "레일·라켓 조준 명령 전송"
        ),
        RuntimeEvent::Failed { track_seq, reason } => {
            warn!(track = track_seq, reason, "실기 단순 제어 실패")
        }
        RuntimeEvent::Done => {}
    }
}

fn log_summary(outcome: &Outcome, cameras: &[CameraStats], estimator: Option<&EstimatorStats>) {
    for stats in cameras {
        info!(
            cam = stats.camera_id,
            frames = stats.frames,
            detections = stats.detections,
            detection_rate = f2(stats.detection_rate()),
            dropped = stats.dropped,
            undistort_failures = stats.undistort_failures,
            "real shot: end — 카메라"
        );
    }
    if let Some(stats) = estimator {
        info!(
            triangulated = stats.triangulated,
            accepted = stats.accepted,
            rejected = stats.rejected,
            seeded = stats.seeded,
            reset = stats.reset,
            skew_p50_ms = stats.skew_percentile(0.50).map(|s| f2(s * 1e3)),
            skew_p95_ms = stats.skew_percentile(0.95).map(|s| f2(s * 1e3)),
            reproj_p50_px = stats.reprojection_percentile(0.50).map(f2),
            reproj_p95_px = stats.reprojection_percentile(0.95).map(f2),
            reprojection_rejected = stats.reprojection_rejected,
            workspace_rejected = stats.workspace_rejected,
            stale_skipped = stats.stale_skipped,
            commit_dropped = stats.commit_dropped,
            preview_dropped = stats.preview_dropped,
            "real shot: end — 추정"
        );
    }
    info!(outcome = outcome.label(), "real shot: end");
}
