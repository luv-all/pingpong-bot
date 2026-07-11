//! # pingpong-app
//!
//! 스레드·채널 오케스트레이션 (plan §4).

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crossbeam_channel::bounded;
use crossbeam_queue::ArrayQueue;
use pingpong_domain::{
    Arm, BallObservation, CameraSource, Detector, DomainError, Estimator, FrameRef, Hardware,
    HitPlane, PixelPoint, Roi, SwingPlanError, Target, Telemetry, TelemetryEvent, constants::table,
    plan_swing,
};

mod arm;
pub use arm::{
    CompetitionUrdfRobot, Robot, RobotMount, UrdfTestRobot, competition_arm, shared_competition_arm,
};
use tracing::{info, info_span, warn};

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
}

impl Default for PipelineConfig {
    fn default() -> Self {
        return Self {
            hit_plane: HitPlane {
                y: table::DEFAULT_HIT_PLANE_Y,
            },
            control_hz: CONTROL_HZ,
            arm: shared_competition_arm(),
        };
    }
}

/// 카메라·추정·제어 스레드를 띄우고 파이프라인을 실행한다.
pub fn run(
    cameras: Vec<Box<dyn CameraSource>>,
    mut estimator: Box<dyn Estimator>,
    mut hardware: Box<dyn Hardware>,
    config: PipelineConfig,
    telemetry: Arc<dyn Telemetry>,
) -> Result<(), PipelineError> {
    let (observation_tx, observation_rx) = bounded::<BallObservation>(OBSERVATION_CHANNEL_CAPACITY);
    let target: Arc<ArrayQueue<Target>> = Arc::new(ArrayQueue::new(1));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut handles: Vec<(PipelineThread, JoinHandle<()>)> = Vec::new();

    for mut camera in cameras {
        let sender = observation_tx.clone();
        handles.push((
            PipelineThread::Camera,
            thread::spawn(move || {
                pin_to_performance_core();
                let mut detector = InlineDetector;
                while let Some((camera_id, frame, timestamp)) = camera.next() {
                    let _span = info_span!("detect", ?camera_id).entered();
                    if let Some(pixel) = detector.detect(frame, roi_for(camera_id)) {
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

    let slot = Arc::clone(&target);
    let telemetry_estimation = Arc::clone(&telemetry);
    let hit_plane = config.hit_plane;
    let shutdown_estimation = Arc::clone(&shutdown);
    handles.push((
        PipelineThread::Estimation,
        thread::spawn(move || {
            pin_to_performance_core();
            while let Ok(observation) = observation_rx.recv() {
                let _span = info_span!("estimator").entered();
                estimator.update(observation);
                if let Some(prediction) = estimator.predict_to(hit_plane) {
                    telemetry_estimation.log(TelemetryEvent::Prediction(prediction));
                    let _ = slot.force_push(Target { prediction });
                }
            }
            shutdown_estimation.store(true, Ordering::Release);
        }),
    ));

    let slot = Arc::clone(&target);
    let telemetry_control = Arc::clone(&telemetry);
    let shutdown_control = Arc::clone(&shutdown);
    let arm = Arc::clone(&config.arm);
    let tick = Duration::from_secs_f64(1.0 / config.control_hz);
    handles.push((
        PipelineThread::Control,
        thread::spawn(move || {
            pin_to_performance_core();
            loop {
                if let Some(target) = slot.pop() {
                    let _span = info_span!("control").entered();
                    if hardware.is_busy() {
                        // sim 물리 스레드가 이미 plan_swing 중 — 늦은 예측으로 InsufficientTime 스팸 방지
                        continue;
                    }
                    let start = match hardware.read_pose() {
                        Ok(pose) => pose,
                        Err(error) => {
                            warn!(?error, "로봇 포즈 읽기 실패 — 스윙 계획 건너뜀");
                            continue;
                        }
                    };
                    match plan_swing(&arm, target, &start) {
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
                            // 이미 늦은 예측 — 재큐하지 않음
                        }
                        Err(error) => {
                            warn!(%error, "스윙 계획 실패");
                            let _ = slot.force_push(target);
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

struct InlineDetector;

impl Detector for InlineDetector {
    fn detect(&mut self, frame: FrameRef, _roi: Option<Roi>) -> Option<PixelPoint> {
        return frame.pixel();
    }
}

fn roi_for(_camera_id: pingpong_domain::CameraId) -> Option<Roi> {
    return None;
}

fn pin_to_performance_core() {
    // 2단계: core_affinity로 성능 코어(P-core) 고정 (plan §4)
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
