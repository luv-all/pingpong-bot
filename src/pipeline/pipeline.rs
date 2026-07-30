//! 파이프라인 실행 진입점.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::camera;
use crate::detector;
use crate::detector::Detector;
use crate::error::DomainError;
use crate::error::SwingPlanError;
use crate::estimator;
use crate::estimator::Estimator;
use crate::estimator::Prediction;
use crate::hardware::Hardware;
use crate::motion;
use crate::telemetry::{Telemetry, TelemetryEvent};
use crossbeam_channel::bounded;
use crossbeam_queue::ArrayQueue;
use tracing::{info, info_span, warn};

use super::{CameraFeed, PipelineConfig, PipelineError, PipelineThread};

const OBSERVATION_CHANNEL_CAPACITY: usize = 64;

/// 카메라·추정·제어 스레드를 띄우고 파이프라인을 실행한다.
pub fn run(
    cameras: Vec<CameraFeed>,
    mut estimator: Box<dyn Estimator>,
    mut hardware: Box<dyn Hardware>,
    config: PipelineConfig,
    telemetry: Arc<dyn Telemetry>,
) -> Result<(), PipelineError> {
    let (observation_tx, observation_rx) =
        bounded::<detector::Observation>(OBSERVATION_CHANNEL_CAPACITY);
    let predictions: Arc<ArrayQueue<Vec<Prediction>>> = Arc::new(ArrayQueue::new(1));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut handles: Vec<(PipelineThread, JoinHandle<()>)> = Vec::new();

    for feed in cameras {
        let sender = observation_tx.clone();
        handles.push((
            PipelineThread::Camera,
            thread::spawn(move || match feed {
                CameraFeed::Hint(mut camera) => {
                    while let Some((camera_id, hint, timestamp)) = camera.next_hint() {
                        let _span = info_span!("detect", ?camera_id).entered();
                        if let Some(pixel) = Detector::passthrough(hint) {
                            if sender
                                .send(detector::Observation {
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
                        let frame = match Detector::undistort(&frame, &params) {
                            Ok(f) => f,
                            Err(err) => {
                                warn!(%err, "undistort 실패 — 프레임 스킵");
                                continue;
                            }
                        };
                        if let Some(pixel) = detector.detect(&frame) {
                            if sender
                                .send(detector::Observation {
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
            let mut series: Vec<(camera::Id, Vec<detector::Observation>)> = calibration
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

                let refs: Vec<(camera::Id, &[detector::Observation])> = series
                    .iter()
                    .filter(|(_, b)| !b.is_empty())
                    .map(|(id, b)| (*id, b.as_slice()))
                    .collect();
                if refs.len() < calibration.min_cameras_for_triangulation() {
                    continue;
                }

                match estimator::Triangulate::synced(&refs, sync_time, &calibration) {
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
                    match motion::Planner::plan_best(&arm, &candidates, &start) {
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

/// 파이프라인 실행 공개 진입점.
pub struct Pipeline;

impl Pipeline {
    pub fn run(
        cameras: Vec<CameraFeed>,
        estimator: Box<dyn Estimator>,
        hardware: Box<dyn Hardware>,
        config: PipelineConfig,
        telemetry: Arc<dyn Telemetry>,
    ) -> Result<(), PipelineError> {
        return run(cameras, estimator, hardware, config, telemetry);
    }
}
