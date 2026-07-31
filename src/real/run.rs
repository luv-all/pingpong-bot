//! `--mode real` 연속 급구 진입점 — 조립 · 메인 루프 · 요약.

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
    ControlStatus, Options, PacedSource, PreviewEvent, PreviewWindow, ShotEvent, ShutdownGuard,
    control_worker, shutdown_channel, sim_host,
};

/// 카메라 → 추정 버퍼. 실시간이라 크게 잡을 이유가 없다 (밀리면 어차피 버린다).
const VISION_CAPACITY: usize = 8;
const PREVIEW_CAPACITY: usize = 2;
const SIM_CAPACITY: usize = 2;
/// 프리뷰가 없을 때 메인 루프 tick.
const IDLE_TICK: Duration = Duration::from_millis(5);

/// 연속 급구: 스윙 완주·센터 복귀 후 다음 공을 다시 친다.
///
/// 종료는 ESC·`q`(preview) 또는 제어 워커 `Done`(치명 실패·셧다운).  
/// `Committed` / `Infeasible`로 프로세스를 끝내지 않는다.
pub fn run(args: &Args) -> Result<()> {
    let options = Options::from_args(args);
    let robot = robot().context("defaults::robot")?;
    let arm = Arc::clone(&robot.arm);

    let hardware = open_hardware(&options, &arm)?;
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
    // 제어 워커가 계획하는 동안 도착한 새 궤적을 보관했다가,
    // 수신 시 가장 최신 항목만 사용한다.
    let (commit_tx, commit_rx) = bounded(8);
    let (status_tx, status_rx) = unbounded::<ControlStatus>();
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
        status_rx,
        preview_tx,
        sim_tx.clone(),
        event_tx.clone(),
        shutdown.clone(),
    );
    let control_handle = control_worker::spawn(
        Box::new(hardware),
        Arc::clone(&arm),
        options.home,
        commit_rx,
        status_tx,
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

/// 세션 요약용 — 마지막 주목 이벤트 + 본 샷 수.
struct Outcome {
    shots_seen: u64,
    last: LastShot,
}

enum LastShot {
    None,
    Committed,
    Infeasible(String),
    Failed(String),
    TimedOut,
    Quit,
}

impl Outcome {
    fn label(&self) -> String {
        let last = match &self.last {
            LastShot::None => "없음".to_owned(),
            LastShot::Committed => "커밋".to_owned(),
            LastShot::Infeasible(reason) => format!("포기 - {reason}"),
            LastShot::Failed(reason) => format!("실패 - {reason}"),
            LastShot::TimedOut => "타임아웃 - 공이 오지 않음".to_owned(),
            LastShot::Quit => "사용자 종료".to_owned(),
        };
        return format!("shots={} last={last}", self.shots_seen);
    }
}

/// 샷 이벤트를 찍고 프리뷰를 돌린다. 세션은 ESC/`q` 또는 제어 `Done`까지 유지.
fn main_loop(
    options: &Options,
    event_rx: &Receiver<ShotEvent>,
    preview_rx: Option<Receiver<PreviewEvent>>,
    guard: ShutdownGuard,
) -> Outcome {
    let mut preview = options.preview.then(|| PreviewWindow::new("real shot"));
    let mut guard = Some(guard);
    let mut wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
    let mut outcome = Outcome {
        shots_seen: 0,
        last: LastShot::None,
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
                ShotEvent::Armed { shot_seq, .. } => {
                    outcome.shots_seen = (*shot_seq).max(outcome.shots_seen);
                    wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
                    timed_out_warned = false;
                }
                ShotEvent::Committed { .. } => outcome.last = LastShot::Committed,
                ShotEvent::Infeasible { reason, .. } => {
                    outcome.last = LastShot::Infeasible(reason.clone());
                }
                ShotEvent::Failed { reason, .. } => {
                    outcome.last = LastShot::Failed(reason.clone());
                }
                ShotEvent::Done => control_done = true,
                _ => {}
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
                "공을 기다리다 시간 초과 — 세션은 유지 (다음 Armed에서 재장전)"
            );
            outcome.last = LastShot::TimedOut;
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
                    outcome.last = LastShot::Quit;
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

/// 샷 결과 HUD (ASCII — Hershey 폰트 제약). 최신 샷으로 덮어쓴다.
fn result_lines(event: &ShotEvent) -> Option<Vec<String>> {
    return match event {
        ShotEvent::Committed {
            shot_seq,
            time_to_impact_secs,
            duration_secs,
            impact,
            rail_start,
            rail_end,
            peak_joint_speed,
        } => Some(vec![
            format!("COMMITTED shot {shot_seq}"),
            format!(
                "impact x{} y{} z{}  tti {}s",
                f2(impact.coords.x),
                f2(impact.coords.y),
                f2(impact.coords.z),
                f2(*time_to_impact_secs)
            ),
            format!(
                "swing  {}s  rail {} -> {}  peak {} rad/s",
                f2(*duration_secs),
                f2(*rail_start),
                f2(*rail_end),
                f2(*peak_joint_speed)
            ),
        ]),
        ShotEvent::Infeasible { shot_seq, reason } => Some(vec![
            format!("ABANDONED shot {shot_seq} - infeasible"),
            reason.clone(),
        ]),
        ShotEvent::Failed { shot_seq, reason } => {
            Some(vec![format!("FAILED shot {shot_seq}"), reason.clone()])
        }
        _ => None,
    };
}

fn open_hardware(options: &Options, arm: &Arc<pingpong_bot::robot::Arm>) -> Result<RealHardware> {
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
        RealHardware::dry_run_with_arm(dxl, Some(rail), Arc::clone(arm))
    } else {
        RealHardware::new(dxl, Some(rail), Arc::clone(arm))
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
/// 커밋 신선도·하드웨어 `stream_hz` 같은 벽시계 로직이 라이브와 같은 조건에서 돈다.
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

fn log_event(event: &ShotEvent) {
    match event {
        ShotEvent::Armed { shot_seq, pose } => info!(
            shot = shot_seq,
            rail_x = f2(pose.rail_x),
            joints = f2_slice(&pose.joints.values),
            "real shot: armed"
        ),
        ShotEvent::Tracking {
            shot_seq,
            position,
            speed,
        } => info!(
            shot = shot_seq,
            x = f2(position.coords.x),
            y = f2(position.coords.y),
            z = f2(position.coords.z),
            speed = f2(*speed),
            "real shot: track"
        ),
        ShotEvent::Committed {
            shot_seq,
            time_to_impact_secs,
            duration_secs,
            impact,
            rail_start,
            rail_end,
            peak_joint_speed,
        } => info!(
            shot = shot_seq,
            duration_secs = f2(*duration_secs),
            rail_start = f2(*rail_start),
            rail_end = f2(*rail_end),
            impact_x = f2(impact.coords.x),
            impact_y = f2(impact.coords.y),
            impact_z = f2(impact.coords.z),
            tti = f2(*time_to_impact_secs),
            peak_joint_speed = f2(*peak_joint_speed),
            "real shot: swing commit"
        ),
        ShotEvent::Infeasible { shot_seq, reason } => {
            info!(
                shot = shot_seq,
                reason, "real shot: 포기 — 관절·토크 한계 (모터 보호)"
            )
        }
        ShotEvent::PlanFailed { shot_seq, reason } => {
            tracing::debug!(shot = shot_seq, reason, "real shot: 계획 실패")
        }
        ShotEvent::Failed { shot_seq, reason } => {
            warn!(shot = shot_seq, reason, "real shot: 실패")
        }
        ShotEvent::Done => {}
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
            stale_skipped = stats.stale_skipped,
            commit_dropped = stats.commit_dropped,
            preview_dropped = stats.preview_dropped,
            "real shot: end — 추정"
        );
    }
    info!(outcome = outcome.label(), "real shot: end");
}
