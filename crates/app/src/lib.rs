//! # pingpong-app
//!
//! 스레드·채널 오케스트레이션 (plan §4).

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::bounded;
use crossbeam_queue::ArrayQueue;
use pingpong_domain::{
    Arm, BallObservation, CameraId, DomainError, Estimator, Hardware, HitPlane, Prediction,
    SwingPlanError, Telemetry, TelemetryEvent, constants::table, in_swing_commit_window, plan_swing,
};
use pingpong_infra::{passthrough_detect, triangulate_synced, Calibration, FrameSource};
use tracing::{info, info_span, warn};

mod arm;
pub use arm::{
    competition_arm, find_robot, robot_ids_csv, shared_competition_arm, RobotEntry,
    DEFAULT_ROBOT_ID, ROBOTS,
};

const OBSERVATION_CHANNEL_CAPACITY: usize = 64;
const CONTROL_HZ: f64 = 100.0;

/// 파이프라인 실행 설정.
pub struct PipelineConfig {
    /// 접수 평면 (공이 맞을 y 깊이)
    pub hit_plane: HitPlane,
    /// 제어 루프 주파수 [Hz]
    pub control_hz: f64,
    /// sim·real 공통 불변 로봇 모델 (plan §2, §7.2)
    pub arm: Arc<Arm>,
    /// 카메라 캘리브 (삼각측량)
    pub calibration: Calibration,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        return Self {
            hit_plane: HitPlane {
                y: table::DEFAULT_HIT_PLANE_Y,
            },
            control_hz: CONTROL_HZ,
            arm: shared_competition_arm(),
            calibration: Calibration::sim(3),
        };
    }
}

/// 카메라·추정·제어 스레드를 띄우고 파이프라인을 실행한다.
pub fn run(
    cameras: Vec<Box<dyn FrameSource>>,
    mut estimator: Box<dyn Estimator>,
    mut hardware: Box<dyn Hardware>,
    config: PipelineConfig,
    telemetry: Arc<dyn Telemetry>,
) -> Result<(), PipelineError> {
    let (observation_tx, observation_rx) = bounded::<BallObservation>(OBSERVATION_CHANNEL_CAPACITY);
    let predictions: Arc<ArrayQueue<Prediction>> = Arc::new(ArrayQueue::new(1));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut handles: Vec<(PipelineThread, JoinHandle<()>)> = Vec::new();

    for mut camera in cameras {
        let sender = observation_tx.clone();
        handles.push((
            PipelineThread::Camera,
            thread::spawn(move || {
                pin_to_performance_core();
                while let Some((camera_id, hint, timestamp)) = camera.next() {
                    let _span = info_span!("detect", ?camera_id).entered();
                    if let Some(pixel) = passthrough_detect(hint) {
                        if sender
                            .send(BallObservation {
                                pixel,
                                camera_id,
                                timestamp,
                            })
                            .is_err()
                        {
                            break;
                        }
                    }
                }
            }),
        ));
    }
    drop(observation_tx);

    let slot = Arc::clone(&predictions);
    let telemetry_estimation = Arc::clone(&telemetry);
    let hit_plane = config.hit_plane;
    let calibration = config.calibration;
    let shutdown_estimation = Arc::clone(&shutdown);
    handles.push((
        PipelineThread::Estimation,
        thread::spawn(move || {
            pin_to_performance_core();
            let mut series: Vec<(CameraId, Vec<BallObservation>)> = calibration
                .cameras
                .iter()
                .map(|c| (c.camera_id, Vec::new()))
                .collect();
            while let Ok(observation) = observation_rx.recv() {
                let _span = info_span!("estimator").entered();
                if let Some((_, buf)) = series
                    .iter_mut()
                    .find(|(id, _)| *id == observation.camera_id)
                {
                    buf.push(observation);
                    // 카메라당 최근 몇 프레임만 유지
                    if buf.len() > 8 {
                        let drain = buf.len() - 8;
                        buf.drain(0..drain);
                    }
                }

                let sync_time = series
                    .iter()
                    .filter_map(|(_, b)| b.last().map(|o| o.timestamp))
                    .max();
                let Some(sync_time) = sync_time else {
                    continue;
                };

                let refs: Vec<(CameraId, &[BallObservation])> = series
                    .iter()
                    .filter(|(_, b)| !b.is_empty())
                    .map(|(id, b)| (*id, b.as_slice()))
                    .collect();
                if refs.len() < calibration.min_cameras_for_triangulation() {
                    continue;
                }

                match triangulate_synced(&refs, sync_time, &calibration) {
                    Ok(point) => {
                        estimator.update(point, sync_time);
                        if let Some(prediction) = estimator.predict_to(hit_plane) {
                            telemetry_estimation.log(TelemetryEvent::Prediction(prediction));
                            let _ = slot.force_push(prediction);
                        }
                    }
                    Err(_) => {
                        // 시야 부족·보간 실패 — 다음 프레임
                    }
                }
            }
            shutdown_estimation.store(true, Ordering::Release);
        }),
    ));

    let slot = Arc::clone(&predictions);
    let telemetry_control = Arc::clone(&telemetry);
    let shutdown_control = Arc::clone(&shutdown);
    let arm = Arc::clone(&config.arm);
    let tick = Duration::from_secs_f64(1.0 / config.control_hz);
    handles.push((
        PipelineThread::Control,
        thread::spawn(move || {
            pin_to_performance_core();
            let mut last_plan_warn = Instant::now() - Duration::from_secs(10);
            loop {
                if let Some(prediction) = slot.pop() {
                    let _span = info_span!("control").entered();
                    if hardware.is_busy() {
                        // sim 물리 스레드가 이미 plan_swing 중 — 늦은 예측으로 InsufficientTime 스팸 방지
                        continue;
                    }
                    if !in_swing_commit_window(prediction.time_to_impact_secs) {
                        continue;
                    }
                    let start = match hardware.read_pose() {
                        Ok(pose) => pose,
                        Err(error) => {
                            warn!(?error, "로봇 포즈 읽기 실패 — 스윙 계획 건너뜀");
                            continue;
                        }
                    };
                    match plan_swing(&arm, prediction, &start) {
                        Ok(trajectory) => {
                            telemetry_control.log(TelemetryEvent::SwingCommand(trajectory.clone()));
                            if let Err(error) = hardware.command(&trajectory) {
                                warn!(
                                    ?error,
                                    duration_secs = trajectory.duration_secs,
                                    "하드웨어 명령 실패"
                                );
                            }
                        }
                        Err(DomainError::InfeasibleSwing(SwingPlanError::InsufficientTime {
                            ..
                        })) => {
                            // 이미 늦은 예측 — 버림
                        }
                        Err(error) => {
                            let now = Instant::now();
                            if now.duration_since(last_plan_warn) >= Duration::from_secs(1) {
                                warn!(%error, "스윙 계획 실패");
                                last_plan_warn = now;
                            }
                        }
                    }
                }

                if shutdown_control.load(Ordering::Acquire) && slot.is_empty() {
                    break;
                }

                thread::sleep(tick);
            }
        }),
    ));

    for (role, handle) in handles {
        handle
            .join()
            .map_err(|_| PipelineError::ThreadPanicked { thread: role })?;
    }

    info!("파이프라인 종료");
    return Ok(());
}

fn pin_to_performance_core() {
    // 나중에 core_affinity 로 P-core 고정
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineThread {
    Camera,
    Estimation,
    Control,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineError {
    ThreadPanicked { thread: PipelineThread },
}

impl std::fmt::Display for PipelineThread {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::Camera => write!(f, "카메라"),
            Self::Estimation => write!(f, "추정"),
            Self::Control => write!(f, "제어"),
        };
    }
}

impl std::fmt::Display for PipelineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::ThreadPanicked { thread } => {
                write!(f, "파이프라인 {thread} 스레드가 패닉했습니다")
            }
        };
    }
}

impl std::error::Error for PipelineError {}
