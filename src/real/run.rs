//! `--mode real` 단발 타격 진입점 — 조립 · 메인 루프 · 요약.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, ensure};
use crossbeam_channel::{Receiver, bounded, unbounded};
use pingpong_bot::camera::{Calibration, CamCliArgs, CamStreamArgs};
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
    Options, PreviewEvent, PreviewWindow, ShotEvent, ShutdownGuard, control_worker,
    shutdown_channel, sim_host,
};

/// 카메라 → 추정 버퍼. 실시간이라 크게 잡을 이유가 없다 (밀리면 어차피 버린다).
const VISION_CAPACITY: usize = 8;
const PREVIEW_CAPACITY: usize = 2;
const SIM_CAPACITY: usize = 2;
/// 프리뷰가 없을 때 메인 루프 tick.
const IDLE_TICK: Duration = Duration::from_millis(5);
/// 샷이 끝난 뒤 제어 워커가 마무리(스윙 완주 + 센터 복귀)할 여유.
const FINISH_GRACE: Duration = Duration::from_secs(15);

/// 공 하나를 받아 스윙 한 번을 커밋하고 멈춘다.
///
/// 창이 있으면(`--preview`) 샷이 끝나도 **프로그램을 끄지 않는다** — 동작만 멈추고
/// 결과를 띄운 채 기다린다. 종료는 ESC·`q`. 창이 없으면 끝나는 즉시 종료한다.
pub fn run(args: &Args) -> Result<()> {
    let options = Options::from_args(args);
    let robot = robot().context("defaults::robot")?;
    let arm = Arc::clone(&robot.arm);

    let hardware = open_hardware(&options, &arm)?;
    let calibration = load_calibration()?;
    let sources = open_cameras()?;
    ensure!(
        sources.len() >= calibration.min_cameras_for_triangulation(),
        "삼각측량에 카메라 {}대가 필요한데 {}대만 열렸다",
        calibration.min_cameras_for_triangulation(),
        sources.len()
    );

    let (guard, shutdown) = shutdown_channel();
    let (vision_tx, vision_rx) = bounded(VISION_CAPACITY);
    let (commit_tx, commit_rx) = bounded(1);
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
        sim_tx,
        event_tx,
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

/// 메인 루프가 끝난 이유.
enum Outcome {
    Committed,
    TooLate,
    Infeasible(String),
    Failed(String),
    TimedOut,
    Quit,
}

impl Outcome {
    fn label(&self) -> String {
        return match self {
            Self::Committed => "커밋".to_owned(),
            Self::TooLate => "포기 - 너무 늦음".to_owned(),
            Self::Infeasible(reason) => format!("포기 - {reason}"),
            Self::Failed(reason) => format!("실패 - {reason}"),
            Self::TimedOut => "타임아웃 - 공이 오지 않음".to_owned(),
            Self::Quit => "사용자 종료".to_owned(),
        };
    }
}

/// 샷 이벤트를 찍고 프리뷰를 돌린다.
///
/// 샷이 끝나면 **셧다운을 걸지 않고** 결과를 화면에 고정한 채 계속 돈다 (창이 있을 때).
/// 카메라·추정은 계속 돌아 화면이 살아 있고, 제어 워커는 이미 래치돼 아무것도 안 한다.
fn main_loop(
    options: &Options,
    event_rx: &Receiver<ShotEvent>,
    preview_rx: Option<Receiver<PreviewEvent>>,
    guard: ShutdownGuard,
) -> Outcome {
    let mut preview = options.preview.then(|| PreviewWindow::new("real shot"));
    let mut guard = Some(guard);
    let wait_deadline = Instant::now() + Duration::from_secs_f64(options.timeout_secs);
    let mut finish_deadline: Option<Instant> = None;
    let mut outcome: Option<Outcome> = None;
    // 창이 없으면 볼 것도 없으니 끝나는 즉시 내려간다.
    let freeze = options.preview;

    let result = loop {
        let mut control_done = false;
        while let Ok(event) = event_rx.try_recv() {
            log_event(&event);
            if let Some(preview) = &mut preview
                && let Some(lines) = result_lines(&event)
            {
                preview.set_result(lines);
            }
            if matches!(event, ShotEvent::Done) {
                control_done = true;
                continue;
            }
            if outcome.is_none() && event.ends_shot() {
                outcome = Some(Outcome::from_event(event));
                finish_deadline = Some(Instant::now() + FINISH_GRACE);
                if !freeze {
                    drop(guard.take());
                }
            }
        }
        // 창 없이 돌 때만 제어 워커 종료가 곧 프로그램 종료다.
        if control_done && !freeze {
            break outcome.unwrap_or(Outcome::Quit);
        }

        if outcome.is_none() && Instant::now() >= wait_deadline {
            warn!(
                timeout_secs = f2(options.timeout_secs),
                "공을 기다리다 시간 초과"
            );
            outcome = Some(Outcome::TimedOut);
            finish_deadline = Some(Instant::now() + FINISH_GRACE);
            if !freeze {
                drop(guard.take());
            }
        }
        if !freeze && finish_deadline.is_some_and(|at| Instant::now() >= at) {
            warn!("제어 워커 마무리 대기 시간 초과");
            break outcome.unwrap_or(Outcome::Quit);
        }

        match &mut preview {
            Some(preview) => {
                if let Some(rx) = &preview_rx {
                    while let Ok(event) = rx.try_recv() {
                        preview.push(event);
                    }
                }
                // show가 waitKey(1)로 루프를 페이싱한다. 종료는 여기서만.
                if preview.show() {
                    break outcome.unwrap_or(Outcome::Quit);
                }
            }
            None => thread::sleep(IDLE_TICK),
        }
    };

    // 여기서 셧다운 — 카메라·추정 워커가 내려간다.
    drop(guard.take());
    if let Some(preview) = &preview {
        preview.close();
    }
    return result;
}

impl Outcome {
    fn from_event(event: ShotEvent) -> Self {
        return match event {
            ShotEvent::Committed { .. } => Self::Committed,
            ShotEvent::TooLate { .. } => Self::TooLate,
            ShotEvent::Infeasible { reason } => Self::Infeasible(reason),
            ShotEvent::Failed { reason } => Self::Failed(reason),
            _ => Self::Quit,
        };
    }
}

/// 샷이 끝난 뒤 화면에 고정할 줄 (ASCII — Hershey 폰트 제약).
fn result_lines(event: &ShotEvent) -> Option<Vec<String>> {
    return match event {
        ShotEvent::Committed {
            time_to_impact_secs,
            duration_secs,
            impact,
            rail_start,
            rail_end,
            peak_joint_speed,
        } => Some(vec![
            "COMMITTED".to_owned(),
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
        ShotEvent::TooLate {
            latest_tti_secs,
            min_swing_secs,
            candidates,
            ball_y,
        } => Some(vec![
            "ABANDONED - too late".to_owned(),
            format!(
                "latest tti {}s < min swing {}s",
                f2(*latest_tti_secs),
                f2(*min_swing_secs)
            ),
            format!("candidates {candidates}  ball y {}", f2(*ball_y)),
        ]),
        ShotEvent::Infeasible { reason } => {
            Some(vec!["ABANDONED - infeasible".to_owned(), reason.clone()])
        }
        ShotEvent::Failed { reason } => Some(vec!["FAILED".to_owned(), reason.clone()]),
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

fn open_cameras() -> Result<OpenedCameras> {
    let cams = CamCliArgs {
        cam: DEFAULT_STEREO_CAM_ROLES.to_vec(),
        stream: CamStreamArgs::default(),
    };
    return cams
        .open_sources()
        .map_err(anyhow::Error::msg)
        .context("실캠 열기");
}

fn log_event(event: &ShotEvent) {
    match event {
        ShotEvent::Armed { pose } => info!(
            rail_x = f2(pose.rail_x),
            joints = f2_slice(&pose.joints.values),
            "real shot: armed"
        ),
        ShotEvent::Tracking { position, speed } => info!(
            x = f2(position.coords.x),
            y = f2(position.coords.y),
            z = f2(position.coords.z),
            speed = f2(*speed),
            "real shot: track"
        ),
        // 필드는 sim `"shot: swing commit"`과 동일 — sim ↔ real 로그를 그대로 비교할 수 있다.
        ShotEvent::Committed {
            time_to_impact_secs,
            duration_secs,
            impact,
            rail_start,
            rail_end,
            peak_joint_speed,
        } => info!(
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
        // 포기 근거 수치를 info로 — `--debug` 없이도 왜 못 쳤는지 보여야 한다.
        ShotEvent::TooLate {
            latest_tti_secs,
            min_swing_secs,
            candidates,
            ball_y,
        } => info!(
            latest_tti = f2(*latest_tti_secs),
            min_swing_secs = f2(*min_swing_secs),
            shortfall = f2(min_swing_secs - latest_tti_secs),
            candidates,
            ball_y = f2(*ball_y),
            "real shot: 포기 — 남은 시간이 최소 스윙 시간 미만"
        ),
        ShotEvent::Infeasible { reason } => {
            info!(reason, "real shot: 포기 — 관절·토크 한계 (모터 보호)")
        }
        ShotEvent::PlanFailed { reason } => tracing::debug!(reason, "real shot: 계획 실패"),
        ShotEvent::Failed { reason } => warn!(reason, "real shot: 실패"),
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
            commit_dropped = stats.commit_dropped,
            preview_dropped = stats.preview_dropped,
            "real shot: end — 추정"
        );
    }
    info!(outcome = outcome.label(), "real shot: end");
}
