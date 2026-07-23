//! 런타임 파이프라인 오케스트레이션.
//!
//! 스레드·채널 오케스트레이션 (plan §4).

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::camera::{CameraParams, FrameSource, HintSource};
use crate::detector::{BallDetector, passthrough_detect, undistort_frame};
use crate::{
    BallObservation, CameraId, DomainError, Estimator, Hardware, InterceptWindow, Prediction,
    Robot, SwingPlanError, Telemetry, TelemetryEvent, plan_best_swing,
};
use crate::{Calibration, triangulate_synced};
use crossbeam_channel::bounded;
use crossbeam_queue::ArrayQueue;
use tracing::{info, info_span, warn};

use crate::defaults::shared_robot;

const OBSERVATION_CHANNEL_CAPACITY: usize = 64;
const CONTROL_HZ: f64 = 100.0;

/// 파이프라인 실행 설정.
pub struct PipelineConfig {
    /// 실제 도달 가능한 타격점을 탐색할 y 구간.
    pub intercept: InterceptWindow,
    /// 제어 루프 주파수 [Hz]
    pub control_hz: f64,
    /// sim·real 공통 불변 로봇 모델 (plan §2, §7.2)
    pub robot: Arc<Robot>,
    /// 카메라 캘리브 (삼각측량)
    pub calibration: Calibration,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        return Self {
            intercept: crate::defaults::intercept(),
            control_hz: CONTROL_HZ,
            robot: shared_robot(),
            calibration: Calibration::sim(3),
        };
    }
}

/// 카메라 입력: sim 힌트 또는 실캠 프레임+검출.
pub enum CameraFeed {
    /// sim — 투영 픽셀을 그대로 observation으로.
    Hint(Box<dyn HintSource>),
    /// 실물 — capture → undistort → detect.
    Detect {
        source: Box<dyn FrameSource>,
        detector: Box<dyn BallDetector>,
        params: CameraParams,
    },
}

/// 카메라·추정·제어 스레드를 띄우고 파이프라인을 실행한다.
pub fn run(
    cameras: Vec<CameraFeed>,
    mut estimator: Box<dyn Estimator>,
    mut hardware: Box<dyn Hardware>,
    config: PipelineConfig,
    telemetry: Arc<dyn Telemetry>,
) -> Result<(), PipelineError> {
    let (observation_tx, observation_rx) = bounded::<BallObservation>(OBSERVATION_CHANNEL_CAPACITY);
    let predictions: Arc<ArrayQueue<Vec<Prediction>>> = Arc::new(ArrayQueue::new(1));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut handles: Vec<(PipelineThread, JoinHandle<()>)> = Vec::new();

    for feed in cameras {
        let sender = observation_tx.clone();
        handles.push((
            PipelineThread::Camera,
            thread::spawn(move || {

                match feed {
                    CameraFeed::Hint(mut camera) => {
                        while let Some((camera_id, hint, timestamp)) = camera.next_hint() {
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
                    }
                    CameraFeed::Detect {
                        mut source,
                        mut detector,
                        params,
                    } => {
                        while let Some(frame) = source.next_frame() {
                            let camera_id = frame.camera_id;
                            let _span = info_span!("detect", ?camera_id).entered();
                            let frame = match undistort_frame(&frame, &params) {
                                Ok(f) => f,
                                Err(err) => {
                                    warn!(%err, "undistort 실패 — 프레임 스킵");
                                    continue;
                                }
                            };
                            if let Some(pixel) = detector.detect(&frame) {
                                if sender
                                    .send(BallObservation {
                                        pixel,
                                        camera_id,
                                        timestamp: frame.timestamp,
                                    })
                                    .is_err()
                                {
                                    break;
                                }
                            }
                        }
                    }
                }
            }),
        ));
    }
    drop(observation_tx);

    let slot = Arc::clone(&predictions);
    let telemetry_estimation = Arc::clone(&telemetry);
    let intercept = config.intercept;
    let calibration = config.calibration;
    let shutdown_estimation = Arc::clone(&shutdown);
    handles.push((
        PipelineThread::Estimation,
        thread::spawn(move || {
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
                        let candidates: Vec<Prediction> = intercept
                            .hit_planes()
                            .into_iter()
                            .filter_map(|plane| estimator.predict_to(plane))
                            .inspect(|prediction| {
                                telemetry_estimation.log(TelemetryEvent::Prediction(*prediction));
                            })
                            .collect();
                        if !candidates.is_empty() {
                            let _ = slot.force_push(candidates);
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
    let arm = Arc::clone(&config.robot.arm);
    let tick = Duration::from_secs_f64(1.0 / config.control_hz);
    handles.push((
        PipelineThread::Control,
        thread::spawn(move || {
            let mut last_plan_warn = Instant::now() - Duration::from_secs(10);
            loop {
                if let Some(candidates) = slot.pop() {
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
                    match plan_best_swing(&arm, &candidates, &start) {
                        Ok(planned) => {
                            let trajectory = planned.trajectory;
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
