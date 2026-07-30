//! 추정 워커 — 삼각측량 → EKF → 접수 평면 예측 → 커밋 게이트.
//!
//! `Ekf`와 `Calibration`을 **단독 소유**한다. 로봇 포즈는 볼 수 없다 (제어 워커만 안다).

use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, TrySendError};
use pingpong_bot::camera;
use pingpong_bot::camera::Calibration;
use pingpong_bot::estimator::{Ekf, Estimator, GateOutcome, Prediction, Triangulate};
use pingpong_bot::robot::motion::{InterceptWindow, Planner};
use tracing::{debug, info_span};

use super::fmt::f2;
use super::{
    CommitRequest, Decision, PreviewEvent, ShotEvent, Shutdown, SimUpdate, Throttle, VisionEvent,
    decide,
};

/// 스테레오 쌍으로 인정할 최대 타임스탬프 차 [s].
///
/// UVC 캠은 하드웨어 동기가 없다 (`TODO.md` §3 "멀티캠 동기 — 비범위"). 120 fps 기준 두 프레임
/// 간격까지는 붙여서 삼각측량하고, 실제 skew는 통계로 남겨 얼마나 나쁜지 수치로 본다.
const MAX_STEREO_SKEW_SECS: f64 = 0.020;

const RECV_TIMEOUT: Duration = Duration::from_millis(50);

/// `--debug` 진척 로그 주기.
const PROGRESS_PERIOD: Duration = Duration::from_secs(1);

/// 카메라 1대의 마지막 검출.
struct Track {
    pixel: camera::Pixel,
    at: Instant,
}

/// 종료 요약용 추정 통계.
#[derive(Debug, Clone, Default)]
pub struct EstimatorStats {
    pub triangulated: u64,
    pub skew_samples: Vec<f64>,
    pub accepted: u64,
    pub rejected: u64,
    pub seeded: u64,
    pub reset: u64,
    pub preview_dropped: u64,
    pub commit_dropped: u64,
}

impl EstimatorStats {
    fn record(&mut self, outcome: GateOutcome) {
        match outcome {
            GateOutcome::Accepted => self.accepted += 1,
            GateOutcome::Rejected => self.rejected += 1,
            GateOutcome::Seeded | GateOutcome::VelocitySeeded => self.seeded += 1,
            GateOutcome::Reset => self.reset += 1,
            GateOutcome::Ignored => {}
        }
    }

    /// 정렬된 skew 표본의 백분위수 [s]. 표본이 없으면 `None`.
    pub fn skew_percentile(&self, q: f64) -> Option<f64> {
        if self.skew_samples.is_empty() {
            return None;
        }
        let mut sorted = self.skew_samples.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let index = ((sorted.len() - 1) as f64 * q).round() as usize;
        return sorted.get(index).copied();
    }
}

/// 추정 워커를 띄운다.
pub fn spawn(
    rx: Receiver<VisionEvent>,
    calibration: Calibration,
    intercept: InterceptWindow,
    commit_tx: Sender<CommitRequest>,
    preview_tx: Option<Sender<PreviewEvent>>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
    shutdown: Shutdown,
) -> JoinHandle<EstimatorStats> {
    return thread::spawn(move || {
        let _span = info_span!("estimator").entered();
        let mut stats = EstimatorStats::default();
        let mut ekf = Ekf::default();
        let mut tracks: Vec<(camera::Id, Option<Track>)> = calibration
            .cameras
            .iter()
            .map(|params| (params.camera_id, None))
            .collect();
        let required = calibration.min_cameras_for_triangulation();
        let planes = intercept.hit_planes();
        let mut announced_track = false;
        let mut last_decision: Option<Decision> = None;
        let mut progress = Throttle::new(PROGRESS_PERIOD);

        while !shutdown.is_down() {
            let event = match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(event) => event,
                Err(RecvTimeoutError::Timeout) => continue,
                Err(RecvTimeoutError::Disconnected) => break,
            };

            if let Some(pixel) = event.pixel {
                let at = event.frame.timestamp;
                if let Some(slot) = tracks
                    .iter_mut()
                    .find(|(id, _)| *id == event.frame.camera_id)
                {
                    slot.1 = Some(Track { pixel, at });
                }
            }

            if let Some((point, sync_time, skew)) = fuse(&tracks, required, &calibration) {
                stats.triangulated += 1;
                stats.skew_samples.push(skew);
                let outcome = ekf.update_position(point, sync_time);
                stats.record(outcome);
                // 버려진 측정만 찍는다 — Accepted는 초당 100건이라 요약 카운트로 충분하다.
                if !outcome.used_measurement() || matches!(outcome, GateOutcome::Reset) {
                    debug!(
                        outcome = ?outcome,
                        reject_streak = ekf.reject_streak(),
                        d2 = ekf.last_gate_d2().map(f2),
                        x = f2(point.coords.x),
                        y = f2(point.coords.y),
                        z = f2(point.coords.z),
                        skew_ms = f2(skew * 1e3),
                        "측정 거부"
                    );
                }
            }

            let tracking = ekf.is_tracking();
            if tracking && !announced_track {
                announced_track = true;
                if let (Some(position), Some(velocity)) = (ekf.position(), ekf.velocity()) {
                    let _ = event_tx.send(ShotEvent::Tracking {
                        position,
                        speed: velocity.norm(),
                    });
                }
            }

            let ball_y = ekf.position().map(|position| position.coords.y);
            let predictions: Vec<Prediction> = if tracking {
                planes
                    .iter()
                    .filter_map(|plane| ekf.predict_to(*plane))
                    .collect()
            } else {
                Vec::new()
            };

            let decision = decide(tracking, ball_y, &predictions);
            // 게이트가 **바뀔 때만** 찍는다 — "왜 안 쳤나"를 로그만으로 되짚을 수 있게.
            // 매 틱 찍으면 초당 수백 줄이라 쓸 수 없다.
            if last_decision != Some(decision) {
                last_decision = Some(decision);
                log_transition(&ekf, decision, &predictions);
            }
            if progress.ready() {
                debug!(
                    triangulated = stats.triangulated,
                    accepted = stats.accepted,
                    rejected = stats.rejected,
                    tracking,
                    decision = ?decision,
                    "추정 진척"
                );
            }

            // 화면·sim에 보여줄 대표 후보 — 커밋 창 안의 첫 후보, 없으면 가장 이른 것.
            let shown = display_candidate(&predictions);

            match decision {
                Decision::Attempt => {
                    let request = CommitRequest {
                        predictions,
                        ball_y: ball_y.unwrap_or(f64::NAN),
                        at: Instant::now(),
                    };
                    // 제어 워커가 아직 앞 요청을 계획 중이면 버린다 — 어차피 더 새 예측이 곧 온다.
                    if let Err(TrySendError::Full(_)) = commit_tx.try_send(request) {
                        stats.commit_dropped += 1;
                    }
                }
                Decision::Wait(_) => {}
            }

            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    ball: ekf.position(),
                    impact: shown.map(|prediction| prediction.impact_position),
                    ..SimUpdate::default()
                });
            }

            if let Some(preview_tx) = &preview_tx {
                // 예측 도달점을 **이 카메라로 재투영**한다. 자르지 않고 프레임 밖인지만
                // 따로 알려 준다 — 안 보이는 이유가 "예측 없음"인지 "화각 밖"인지 구분해야
                // 벤치에서 진단이 된다 (접수 평면 y 0.08~0.35는 로봇 코앞이라 화각을
                // 벗어나기 쉽다).
                let params = calibration.params(event.frame.camera_id);
                let impact_pixel = shown.zip(params).and_then(|(prediction, params)| {
                    params.project_world_unclipped(prediction.impact_position)
                });
                let impact_offscreen = impact_pixel.is_some_and(|pixel| {
                    params.is_some_and(|params| {
                        pixel.x < 0.0
                            || pixel.y < 0.0
                            || pixel.x >= f64::from(params.width)
                            || pixel.y >= f64::from(params.height)
                    })
                });
                let hud = hud_lines(&ekf, &decision, shown.as_ref(), impact_offscreen);
                let preview = PreviewEvent {
                    frame: event.frame,
                    pixel: event.pixel,
                    impact_pixel,
                    impact_offscreen,
                    hud,
                };
                if let Err(TrySendError::Full(_)) = preview_tx.try_send(preview) {
                    stats.preview_dropped += 1;
                }
            }
        }

        debug!(
            triangulated = stats.triangulated,
            accepted = stats.accepted,
            "추정 워커 종료"
        );
        return stats;
    });
}

/// 충분히 동시각인 검출만 모아 삼각측량한다. `(점, 동기 시각, skew [s])`.
fn fuse(
    tracks: &[(camera::Id, Option<Track>)],
    required: usize,
    calibration: &Calibration,
) -> Option<(pingpong_bot::Point3, Instant, f64)> {
    let newest = tracks
        .iter()
        .filter_map(|(_, track)| track.as_ref().map(|t| t.at))
        .max()?;

    let mut hits: Vec<(camera::Id, camera::Pixel)> = Vec::with_capacity(tracks.len());
    let mut oldest = newest;
    for (id, track) in tracks {
        let Some(track) = track else { continue };
        if newest.duration_since(track.at).as_secs_f64() > MAX_STEREO_SKEW_SECS {
            continue;
        }
        oldest = oldest.min(track.at);
        hits.push((*id, track.pixel));
    }
    if hits.len() < required {
        return None;
    }

    let point = Triangulate::pixels(&hits, calibration)?;
    let skew = newest.duration_since(oldest).as_secs_f64();
    return Some((point, newest, skew));
}

/// 게이트가 바뀐 순간의 스냅샷. 커밋까지 못 간 샷을 로그만으로 진단하는 근거다.
fn log_transition(ekf: &Ekf, decision: Decision, predictions: &[Prediction]) {
    let (tti_min, tti_max) = predictions
        .iter()
        .map(|prediction| prediction.time_to_impact_secs)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
            (lo.min(t), hi.max(t))
        });
    let position = ekf.position();
    debug!(
        decision = ?decision,
        candidates = predictions.len(),
        tti_min = (!predictions.is_empty()).then(|| f2(tti_min)),
        tti_max = (!predictions.is_empty()).then(|| f2(tti_max)),
        ball_x = position.map(|p| f2(p.coords.x)),
        ball_y = position.map(|p| f2(p.coords.y)),
        ball_z = position.map(|p| f2(p.coords.z)),
        speed = ekf.velocity().map(|v| f2(v.norm())),
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

/// HUD 문자열은 **ASCII만** 쓴다 — Hershey 폰트가 유니코드를 못 그린다 (한글은 `??????`).
/// 한글 사유는 `ShotEvent` 로그로만 나간다.
fn hud_lines(
    ekf: &Ekf,
    decision: &Decision,
    shown: Option<&Prediction>,
    impact_offscreen: bool,
) -> Vec<String> {
    let state = match decision {
        Decision::Attempt => "ATTEMPT plan request".to_owned(),
        Decision::Wait(reason) => reason.label().to_owned(),
    };
    let mut lines = vec![state];
    if let Some(position) = ekf.position() {
        let speed = ekf.velocity().map(|v| v.norm()).unwrap_or(0.0);
        lines.push(format!(
            "ball   x{:+.2} y{:+.2} z{:+.2}  |v|{:.1}",
            position.coords.x, position.coords.y, position.coords.z, speed
        ));
    }
    if let Some(prediction) = shown {
        let offscreen = if impact_offscreen {
            "  [OFF-FRAME]"
        } else {
            ""
        };
        lines.push(format!(
            "impact x{:+.2} y{:+.2} z{:+.2}  tti {:.2}s{offscreen}",
            prediction.impact_position.coords.x,
            prediction.impact_position.coords.y,
            prediction.impact_position.coords.z,
            prediction.time_to_impact_secs
        ));
    } else {
        lines.push("impact none".to_owned());
    }
    if let Some(d2) = ekf.last_gate_d2() {
        lines.push(format!("gate   d2 {d2:.1}  reject {}", ekf.reject_streak()));
    }
    return lines;
}
