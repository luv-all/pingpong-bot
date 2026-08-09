//! 새 비전 워커 — 카메라별 후보 픽셀을 일괄 탄도 적합해 궤적 계약으로 전달한다.
//!
//! 접수 평면과 로봇 도달성은 여기서 판단하지 않는다. 이 워커는
//! [`pingpong_bot::vision::Trajectory`] 전체를 제어 워커에 넘기기만 한다.

use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, TrySendError};
use pingpong_bot::camera::Calibration;
use pingpong_bot::vision::{Fit, Outcome};
use tracing::{debug, info_span};

use super::fmt::f2;
use super::{
    CommitRequest, PreviewEvent, RuntimeEvent, Shutdown, SimUpdate, Throttle, VisionEvent,
};

const RECV_TIMEOUT: Duration = Duration::from_millis(50);
const PROGRESS_PERIOD: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Default)]
pub struct EstimatorStats {
    pub accepted: u64,
    pub rejected: u64,
    pub seeded: u64,
    pub preview_dropped: u64,
    pub commit_dropped: u64,
}

impl EstimatorStats {
    fn record(&mut self, outcome: Outcome) {
        match outcome {
            Outcome::Accepted => self.accepted += 1,
            Outcome::Rejected { .. } => self.rejected += 1,
            Outcome::Seeded => self.seeded += 1,
            Outcome::Idle => {}
        }
    }
}

/// 픽셀 적합과 트리거는 `defaults::vision`의 본선 설정만 사용한다.
pub fn spawn(
    rx: Receiver<VisionEvent>,
    calibration: Calibration,
    commit_tx: Sender<CommitRequest>,
    commit_evict_rx: Receiver<CommitRequest>,
    preview_tx: Option<Sender<PreviewEvent>>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<RuntimeEvent>,
    shutdown: Shutdown,
) -> JoinHandle<EstimatorStats> {
    return thread::spawn(move || {
        let _span = info_span!("vision-fit").entered();
        let mut stats = EstimatorStats::default();
        let mut fit = Fit::new(&calibration, pingpong_bot::defaults::vision::trigger());
        let mut origin: Option<Instant> = None;
        let mut announced_seq: Option<u64> = None;
        let mut progress = Throttle::new(PROGRESS_PERIOD);

        while !shutdown.is_down() {
            let event = match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(event) => event,
                Err(RecvTimeoutError::Timeout) => continue,
                Err(RecvTimeoutError::Disconnected) => break,
            };

            let base = *origin.get_or_insert(event.frame.timestamp);
            let t = event.frame.timestamp.saturating_duration_since(base);
            let outcome = fit.observe(event.frame.camera_id, event.found, t);
            stats.record(outcome);
            if let Outcome::Rejected { px } = outcome {
                debug!(
                    cam = event.frame.camera_id.0,
                    residual_px = f2(px),
                    "비전 관측 제외"
                );
            }

            let measured = fit.measured().last().copied();
            if let Some(state) = measured
                && announced_seq != Some(fit.seq())
            {
                announced_seq = Some(fit.seq());
                let _ = event_tx.send(RuntimeEvent::Tracking {
                    track_seq: fit.seq(),
                    position: state.position,
                    speed: state.velocity.norm(),
                });
            }

            let trajectory = fit.trajectory(base);
            if let Some(trajectory) = trajectory.clone() {
                send_latest_commit(
                    &commit_tx,
                    &commit_evict_rx,
                    CommitRequest {
                        trajectory,
                        at: Instant::now(),
                    },
                    &mut stats,
                );
            }

            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    ball: measured.map(|state| state.position),
                    ..SimUpdate::default()
                });
            }

            if progress.ready() {
                debug!(
                    accepted = stats.accepted,
                    rejected = stats.rejected,
                    seeded = stats.seeded,
                    sightings = fit.sightings(),
                    seq = fit.seq(),
                    predicting = trajectory.is_some(),
                    "새 비전 적합 진척"
                );
            }

            if let Some(preview_tx) = &preview_tx {
                let params = calibration.params(event.frame.camera_id);
                // 흰 점은 새 Fit의 현재 상태 재투영이다. 구 EKF/삼각측량 값이 아니다.
                let fitted_pixel = measured
                    .zip(params)
                    .and_then(|(state, params)| params.project_world_unclipped(state.position));
                let hud = hud_lines(&fit, measured.as_ref(), trajectory.as_ref());
                let preview = PreviewEvent {
                    frame: event.frame,
                    pixel: event.found.map(|candidate| candidate.pixel),
                    target_pixel: None,
                    target_offscreen: false,
                    fitted_pixel,
                    hud,
                };
                if let Err(TrySendError::Full(_)) = preview_tx.try_send(preview) {
                    stats.preview_dropped += 1;
                }
            }
        }

        debug!(
            accepted = stats.accepted,
            seeded = stats.seeded,
            "새 비전 워커 종료"
        );
        return stats;
    });
}

fn hud_lines(
    fit: &Fit,
    measured: Option<&pingpong_bot::vision::State>,
    trajectory: Option<&pingpong_bot::vision::Trajectory>,
) -> Vec<String> {
    let mut lines = vec![format!(
        "VISION seq {} sightings {} predicted {}",
        fit.seq(),
        fit.sightings(),
        trajectory.map_or(0, |trajectory| trajectory.predicted.len())
    )];
    if let Some(state) = measured {
        lines.push(format!(
            "ball x{} y{} z{} speed{}",
            f2(state.position.x),
            f2(state.position.y),
            f2(state.position.z),
            f2(state.velocity.norm())
        ));
        lines.push(format!(
            "sigma cm x{} y{} z{}",
            f2(state.sigma_position.x * 100.0),
            f2(state.sigma_position.y * 100.0),
            f2(state.sigma_position.z * 100.0)
        ));
    }
    return lines;
}

fn send_latest_commit(
    tx: &Sender<CommitRequest>,
    evict_rx: &Receiver<CommitRequest>,
    request: CommitRequest,
    stats: &mut EstimatorStats,
) {
    match tx.try_send(request) {
        Ok(()) => {}
        Err(TrySendError::Full(request)) => {
            let _ = evict_rx.try_recv();
            if tx.try_send(request).is_err() {
                stats.commit_dropped += 1;
            }
        }
        Err(TrySendError::Disconnected(_)) => {}
    }
}
