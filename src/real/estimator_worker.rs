//! 추정 워커 — 검출 픽셀 → EKF → 접수 평면 예측 → 커밋 게이트.
//!
//! [`Ekf`]를 **단독 소유**한다. 로봇 포즈는 볼 수 없다 (제어 워커만 안다).
//!
//! 삼각측량은 시드 한 번뿐이다. 그 뒤로는 카메라가 서로를 기다리지 않고 각자 자기 시각의
//! 픽셀로 필터를 보정한다 — 짝 맞추기, 보간 지연, `stale_skipped` 가 전부 사라졌다.

use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, TrySendError};
use pingpong_bot::camera::Calibration;
use pingpong_bot::constants::table;
use pingpong_bot::defaults::EstimatorParams;
use pingpong_bot::estimator::{HitPlane, Prediction};
use pingpong_bot::robot::motion::{InterceptWindow, Planner};
use pingpong_bot::vision::{Ekf, Outcome, State, Trajectory, Trigger, triggers};
use tracing::{debug, info_span};

use super::fmt::f2;
use super::{
    CommitRequest, ControlStatus, Decision, PreviewEvent, ShotEvent, Shutdown, SimUpdate, Throttle,
    TrackRequest, VisionEvent, decide,
};

const RECV_TIMEOUT: Duration = Duration::from_millis(50);

/// `--debug` 진척 로그 주기.
const PROGRESS_PERIOD: Duration = Duration::from_secs(1);

/// 종료 요약용 추정 통계.
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

/// 실기 트리거 — 필터가 좁혀졌거나, 늦어도 네트를 넘으면.
///
/// [`triggers::Any`]인 이유는 둘 중 하나만 쓰면 하나를 포기해야 해서다. σ만 보면 검출이
/// 나쁜 샷에서 영영 안 걸리고, 평면만 보면 이미 확신이 선 샷도 네트까지 기다린다.
fn default_trigger(params: &EstimatorParams) -> Box<dyn Trigger> {
    let sigma = params.max_impact_sigma;
    return Box::new(triggers::Any(vec![
        Box::new(triggers::SigmaThreshold {
            position: pingpong_bot::Vector3::repeat(sigma),
            // 속도 σ는 리드타임을 곱해 도달점 오차가 되므로 같은 예산을 최대 리드로 나눈다.
            velocity: pingpong_bot::Vector3::repeat(sigma / params.max_lead),
        }),
        Box::new(triggers::PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    ]));
}

/// 추정 워커를 띄운다.
#[allow(clippy::too_many_arguments)]
pub fn spawn(
    rx: Receiver<VisionEvent>,
    calibration: Calibration,
    intercept: InterceptWindow,
    commit_tx: Sender<CommitRequest>,
    track_tx: Sender<TrackRequest>,
    status_rx: Receiver<ControlStatus>,
    preview_tx: Option<Sender<PreviewEvent>>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
    shutdown: Shutdown,
) -> JoinHandle<EstimatorStats> {
    return thread::spawn(move || {
        let _span = info_span!("estimator").entered();
        let mut stats = EstimatorStats::default();
        let estimator_params = EstimatorParams::default();
        let mut ekf = Ekf::new(&calibration, default_trigger(&estimator_params));
        // 프레임 시각은 벽시계지만 필터는 경과만 안다. 첫 프레임이 t=0 이다.
        let mut origin: Option<Instant> = None;
        let planes = intercept.hit_planes();
        let mut accepting = false;
        let mut shot_seq: u64 = 0;
        let mut announced_seq: Option<u64> = None;
        let mut last_decision: Option<Decision> = None;
        let mut progress = Throttle::new(PROGRESS_PERIOD);

        while !shutdown.is_down() {
            while let Ok(status) = status_rx.try_recv() {
                match status {
                    ControlStatus::Ready { shot_seq: seq } => {
                        accepting = true;
                        shot_seq = seq;
                        announced_seq = None;
                        last_decision = None;
                        ekf.drop_track();
                    }
                    ControlStatus::Recovering { .. } => accepting = false,
                }
            }

            let event = match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(event) => event,
                Err(RecvTimeoutError::Timeout) => continue,
                Err(RecvTimeoutError::Disconnected) => break,
            };

            let base = *origin.get_or_insert(event.frame.timestamp);
            let t = event.frame.timestamp.saturating_duration_since(base);
            let outcome = ekf.observe(event.frame.camera_id, event.found, t);
            stats.record(outcome);
            if let Outcome::Rejected { d2 } = outcome {
                debug!(d2 = f2(d2), cam = event.frame.camera_id.0, "측정 거부");
            }

            let trajectory = ekf.trajectory(base);
            let ball = ekf.measured().last().copied();

            // 트랙이 새로 서면 한 번 알린다. `seq`가 곧 "같은 공인가"의 근거다.
            if let Some(state) = ball
                && announced_seq != Some(ekf.seq())
            {
                announced_seq = Some(ekf.seq());
                let _ = event_tx.send(ShotEvent::Tracking {
                    shot_seq,
                    position: state.position,
                    speed: state.velocity.norm(),
                });
            }

            let predictions = predictions_at(trajectory.as_ref(), &planes);
            let impact_sigma = display_candidate(&predictions)
                .zip(ball)
                .map(|(prediction, state)| impact_sigma(&state, prediction.time_to_impact_secs));
            let decision = decide(
                trajectory.is_some(),
                ball.map(|state| state.position.y),
                &predictions,
                impact_sigma,
            );
            // 게이트가 **바뀔 때만** 찍는다. 매 틱 찍으면 초당 수백 줄이라 쓸 수 없다.
            if last_decision != Some(decision) {
                last_decision = Some(decision);
                log_transition(ball.as_ref(), decision, &predictions);
            }
            if progress.ready() {
                debug!(
                    accepted = stats.accepted,
                    rejected = stats.rejected,
                    seeded = stats.seeded,
                    tracking = trajectory.is_some(),
                    accepting,
                    shot = shot_seq,
                    decision = ?decision,
                    "추정 진척"
                );
            }

            let shown = display_candidate(&predictions);
            match decision {
                Decision::Attempt if accepting => {
                    let request = CommitRequest {
                        predictions,
                        ball_y: ball.map_or(f64::NAN, |state| state.position.y),
                        at: Instant::now(),
                    };
                    // 제어 워커가 아직 앞 요청을 계획 중이면 버린다 — 더 새 예측이 곧 온다.
                    if let Err(TrySendError::Full(_)) = commit_tx.try_send(request) {
                        stats.commit_dropped += 1;
                    }
                }
                // 아직 칠 때가 아니어도 예측이 있으면 제어 워커가 미리 옮길 수 있게 보낸다.
                Decision::Attempt | Decision::Wait(_) => {
                    if !predictions.is_empty() {
                        // 놓쳐도 다음 프레임에 다시 온다.
                        let _ = track_tx.try_send(TrackRequest {
                            predictions,
                            at: Instant::now(),
                        });
                    }
                }
            }

            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    ball: ball.map(|state| state.position),
                    impact: shown.map(|prediction| prediction.impact_position),
                    ..SimUpdate::default()
                });
            }

            if let Some(preview_tx) = &preview_tx {
                // 예측 도달점을 **이 카메라로 재투영**한다. 자르지 않고 프레임 밖인지만 따로
                // 알려 준다 — 안 보이는 이유가 "예측 없음"인지 "화각 밖"인지 구분해야 벤치에서
                // 진단이 된다 (접수 평면 y 0.08~0.35는 로봇 코앞이라 화각을 벗어나기 쉽다).
                let params = calibration.params(event.frame.camera_id);
                let impact_pixel = shown.zip(params).and_then(|(prediction, params)| {
                    params.project_world_unclipped(prediction.impact_position)
                });
                let impact_offscreen = impact_pixel.zip(params).is_some_and(|(pixel, params)| {
                    pixel.x < 0.0
                        || pixel.y < 0.0
                        || pixel.x >= f64::from(params.width)
                        || pixel.y >= f64::from(params.height)
                });
                // 필터 상태를 이 카메라로 되쏜다. 초록(검출)과 벌어진 거리가 곧 잔차다.
                let filtered_pixel = ball
                    .zip(params)
                    .and_then(|(state, params)| params.project_world_unclipped(state.position));
                let preview = PreviewEvent {
                    frame: event.frame,
                    pixel: event.found.map(|candidate| candidate.pixel),
                    impact_pixel,
                    impact_offscreen,
                    raw_pixel: filtered_pixel,
                    hud: hud_lines(
                        ball.as_ref(),
                        &decision,
                        shown.as_ref(),
                        impact_offscreen,
                        impact_sigma,
                    ),
                };
                if let Err(TrySendError::Full(_)) = preview_tx.try_send(preview) {
                    stats.preview_dropped += 1;
                }
            }
        }

        debug!(
            accepted = stats.accepted,
            seeded = stats.seeded,
            "추정 워커 종료"
        );
        return stats;
    });
}

/// 예측 궤적을 각 접수 평면에서 잘라 기구학 계약으로 바꾼다.
///
/// 궤적은 트리거 순간에 한 번 얼고 그 뒤로 갱신되지 않는다. 매 프레임 다시 적분하면
/// 예측이 측정에 끌려가 "예측이 맞았는지"를 볼 수 없다.
fn predictions_at(trajectory: Option<&Trajectory>, planes: &[HitPlane]) -> Vec<Prediction> {
    let Some(trajectory) = trajectory else {
        return Vec::new();
    };
    // 예측 시작 시각 기준의 잔여 시간이어야 제어측 리드타임과 뜻이 같다.
    let Some(now) = trajectory.measured.last().map(|state| state.t) else {
        return Vec::new();
    };
    return planes
        .iter()
        .filter_map(|plane| {
            let state = trajectory.predicted.at_plane(plane.y)?;
            return Some(Prediction {
                time_to_impact_secs: state.t.saturating_sub(now).as_secs_f64(),
                impact_position: state.position,
                incoming_velocity: state.velocity,
            });
        })
        .collect();
}

/// 도달점 불확실성 [m] — 리드가 길수록 σ_v가 크게 실린다.
///
/// 축별 σ를 합칠 수밖에 없는 자리다 (계약이 스칼라 하나를 받는다). 가장 나쁜 축을 쓴다 —
/// 제곱합은 잘 관측되는 축이 나쁜 축을 희석한다.
fn impact_sigma(state: &State, lead: f64) -> f64 {
    let combined = state.sigma_position + state.sigma_velocity * lead.max(0.0);
    return combined.max();
}

/// 게이트가 바뀐 순간의 스냅샷. 커밋까지 못 간 샷을 로그만으로 진단하는 근거다.
fn log_transition(ball: Option<&State>, decision: Decision, predictions: &[Prediction]) {
    let (tti_min, tti_max) = predictions
        .iter()
        .map(|prediction| prediction.time_to_impact_secs)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
            (lo.min(t), hi.max(t))
        });
    debug!(
        decision = ?decision,
        candidates = predictions.len(),
        tti_min = (!predictions.is_empty()).then(|| f2(tti_min)),
        tti_max = (!predictions.is_empty()).then(|| f2(tti_max)),
        ball_x = ball.map(|s| f2(s.position.x)),
        ball_y = ball.map(|s| f2(s.position.y)),
        ball_z = ball.map(|s| f2(s.position.z)),
        speed = ball.map(|s| f2(s.velocity.norm())),
        "real shot: 게이트 전이"
    );
}

/// 화면에 대표로 보여줄 후보 — 커밋 창 안의 첫 후보, 없으면 tti가 가장 이른 것.
///
/// `plan_best`가 실제로 고르는 후보(점수 순)와 반드시 같지는 않다 — 어디를 칠 셈인지
/// 가늠하는 표시일 뿐이다.
fn display_candidate(predictions: &[Prediction]) -> Option<Prediction> {
    return predictions
        .iter()
        .find(|prediction| Planner::in_commit_window(prediction.time_to_impact_secs))
        .or_else(|| {
            predictions.iter().min_by(|a, b| {
                a.time_to_impact_secs
                    .partial_cmp(&b.time_to_impact_secs)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
        })
        .copied();
}

/// HUD 문자열은 ASCII로 쓴다 — Hershey 폰트가 유니코드를 못 그린다.
/// 한글 사유는 `ShotEvent` 로그로만 나간다.
fn hud_lines(
    ball: Option<&State>,
    decision: &Decision,
    shown: Option<&Prediction>,
    impact_offscreen: bool,
    impact_sigma: Option<f64>,
) -> Vec<String> {
    let mut lines = vec![match decision {
        Decision::Attempt => "ATTEMPT plan request".to_owned(),
        Decision::Wait(reason) => reason.label().to_owned(),
    }];
    if let Some(state) = ball {
        lines.push(format!(
            "ball   x{:+.2} y{:+.2} z{:+.2}  |v|{:.1}",
            state.position.x,
            state.position.y,
            state.position.z,
            state.velocity.norm()
        ));
    }
    match shown {
        Some(prediction) => {
            let offscreen = if impact_offscreen {
                "  [OFF-FRAME]"
            } else {
                ""
            };
            lines.push(format!(
                "impact x{:+.2} y{:+.2} z{:+.2}  tti {:.2}s{offscreen}",
                prediction.impact_position.x,
                prediction.impact_position.y,
                prediction.impact_position.z,
                prediction.time_to_impact_secs
            ));
        }
        None => lines.push("impact none".to_owned()),
    }
    if let Some(state) = ball {
        lines.push(format!(
            "sigma  p {:.0}/{:.0}/{:.0} cm  v {:.1}/{:.1}/{:.1}",
            state.sigma_position.x * 100.0,
            state.sigma_position.y * 100.0,
            state.sigma_position.z * 100.0,
            state.sigma_velocity.x,
            state.sigma_velocity.y,
            state.sigma_velocity.z,
        ));
    }
    if let Some(sigma) = impact_sigma {
        lines.push(format!(
            "impact sigma {:.0} cm (limit {:.0})",
            sigma * 100.0,
            EstimatorParams::default().max_impact_sigma * 100.0
        ));
    }
    return lines;
}
