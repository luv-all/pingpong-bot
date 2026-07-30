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
use super::{
    Options, PreviewEvent, PreviewWindow, ShotEvent, ShutdownGuard, control_worker,
    shutdown_channel,
};

/// 카메라 → 추정 버퍼. 실시간이라 크게 잡을 이유가 없다 (밀리면 어차피 버린다).
const VISION_CAPACITY: usize = 8;
const PREVIEW_CAPACITY: usize = 2;
/// 프리뷰가 없을 때 메인 루프 tick.
const IDLE_TICK: Duration = Duration::from_millis(5);
/// 커밋/포기 후 제어 워커가 마무리(스윙 완주 + 센터 복귀)할 여유.
const FINISH_GRACE: Duration = Duration::from_secs(15);

/// 공 하나를 받아 스윙 한 번을 커밋하고 종료한다.
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
        event_tx.clone(),
        shutdown.clone(),
    );
    let control_handle = control_worker::spawn(
        Box::new(hardware),
        Arc::clone(&arm),
        options.home,
        commit_rx,
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

    log_summary(&outcome, &camera_stats, estimator_stats.as_ref());
    return Ok(());
}

/// 메인 루프가 끝난 이유.
enum Outcome {
    Committed,
    Abandoned(String),
    Failed(String),
    TimedOut,
    Quit,
}

impl Outcome {
    fn label(&self) -> String {
        return match self {
            Self::Committed => "커밋".to_owned(),
            Self::Abandoned(reason) => format!("포기 - {reason}"),
            Self::Failed(reason) => format!("실패 - {reason}"),
            Self::TimedOut => "타임아웃 - 공이 오지 않음".to_owned(),
            Self::Quit => "사용자 종료".to_owned(),
        };
    }
}

/// 샷 이벤트를 찍고 프리뷰를 돌린다. 종료가 확정되면 셧다운을 걸고 `Done`까지 기다린다.
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

    let result = loop {
        // 제어 워커가 마무리(스윙 완주 + 센터 복귀)까지 끝냈는가 — 유일한 정상 종료.
        let mut done = false;
        while let Ok(event) = event_rx.try_recv() {
            log_event(&event);
            if matches!(event, ShotEvent::Done) {
                done = true;
                break;
            }
            if outcome.is_none() && event.ends_shot() {
                outcome = Some(match event {
                    ShotEvent::Committed { .. } => Outcome::Committed,
                    ShotEvent::Abandoned { reason } => Outcome::Abandoned(reason),
                    ShotEvent::Failed { reason } => Outcome::Failed(reason),
                    _ => Outcome::Quit,
                });
                finish_deadline = Some(Instant::now() + FINISH_GRACE);
                drop(guard.take());
            }
        }
        if done {
            break outcome.unwrap_or(Outcome::Quit);
        }

        if outcome.is_none() && Instant::now() >= wait_deadline {
            warn!(
                timeout_secs = options.timeout_secs,
                "공을 기다리다 시간 초과 — 종료"
            );
            outcome = Some(Outcome::TimedOut);
            finish_deadline = Some(Instant::now() + FINISH_GRACE);
            drop(guard.take());
        }
        if finish_deadline.is_some_and(|at| Instant::now() >= at) {
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
                // show가 waitKey(1)로 루프를 페이싱한다.
                if preview.show() && outcome.is_none() {
                    outcome = Some(Outcome::Quit);
                    finish_deadline = Some(Instant::now() + FINISH_GRACE);
                    drop(guard.take());
                }
            }
            None => thread::sleep(IDLE_TICK),
        }
    };

    if let Some(preview) = &preview {
        preview.close();
    }
    return result;
}

fn open_hardware(options: &Options, arm: &Arc<pingpong_bot::robot::Arm>) -> Result<RealHardware> {
    let mut dxl = DynamixelConfig::default();
    if let Some(port) = &options.dxl_port {
        dxl.port = port.clone();
    }
    let rail = RailConfig::default();
    info!(
        port = %dxl.port,
        dry_run = options.dry_run,
        rail_enabled = rail.enabled,
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
            rail_x = pose.rail_x,
            joints = ?pose.joints.values,
            "real shot: armed"
        ),
        ShotEvent::Tracking { position, speed } => info!(
            x = position.coords.x,
            y = position.coords.y,
            z = position.coords.z,
            speed,
            "real shot: track"
        ),
        // 필드는 sim `"shot: swing commit"`과 동일 — sim ↔ real 로그를 그대로 비교할 수 있다.
        ShotEvent::Committed {
            time_to_impact_secs,
            duration_secs,
            impact,
            rail_end,
            peak_joint_speed,
        } => info!(
            duration_secs,
            rail_end,
            impact_x = impact.coords.x,
            impact_y = impact.coords.y,
            impact_z = impact.coords.z,
            tti = time_to_impact_secs,
            peak_joint_speed,
            "real shot: swing commit"
        ),
        ShotEvent::Abandoned { reason } => info!(reason, "real shot: 포기"),
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
            detection_rate = stats.detection_rate(),
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
            skew_p50_ms = stats.skew_percentile(0.50).map(|s| s * 1e3),
            skew_p95_ms = stats.skew_percentile(0.95).map(|s| s * 1e3),
            commit_dropped = stats.commit_dropped,
            preview_dropped = stats.preview_dropped,
            "real shot: end — 추정"
        );
    }
    info!(outcome = outcome.label(), "real shot: end");
}
