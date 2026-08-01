//! 클립 한 번 훑기 — 검출·삼각측량으로 **실제 궤적**을 만들고, 그 위에서 EKF를 재생해
//! 매 프레임의 **예측 궤적**을 남긴다.
//!
//! 삼각측량 쌍은 **프레임 인덱스**로 맞춘다. 두 AVI는 동시 녹화라 같은 인덱스가 같은 시각이고,
//! 여기는 "실제로 어디였나"를 만드는 자리라 런타임의 타임스탬프 보간보다 인덱스가 정확하다
//! (`diag_clip_prediction`과 같은 규칙).
//!
//! 두 번 훑지 않는다 — 검출은 여기서 한 번만 하고, 재생 루프는 그 결과를 다시 쓴다.
//! 되감아도 검출이 다시 돌지 않으므로 0.1x로 앞뒤를 오가도 값이 흔들리지 않는다.

use std::path::Path;
use std::time::{Duration, Instant};

use nalgebra::Vector3;
use pingpong_bot::Point3;
use pingpong_bot::camera::{self, Calibration, FrameSource, OpenCvCapture};
use pingpong_bot::constants::table;
use pingpong_bot::defaults::{self, PhysicsParams};
use pingpong_bot::estimator::{
    Decision, Ekf, Estimator, GateOutcome, Kinematics, Prediction, Triangulate, decide,
};
use pingpong_bot::robot::motion::{InterceptWindow, Planner};

/// 재투영 오차가 이보다 크면 두 캠이 서로 다른 걸 잡은 것 — 런타임과 같은 상한.
const MAX_REPROJECTION_PX: f64 = 14.0;

/// 예측 궤적 적분 스텝 [s] — `EstimatorParams::integrate_dt`와 같다.
const INTEGRATE_DT: f64 = 0.001;

/// 예측 궤적 상한 [s]. 플레이 부피를 못 벗어나도 여기서 끊는다.
const HORIZON_SECS: f64 = 2.0;

/// 궤적 표본 간격 [s] — 그리기·비교용. 1 ms를 전부 담을 이유가 없다.
const SAMPLE_DT: f64 = 0.005;

/// 예측을 끊는 플레이 부피 여유 [m].
const VOLUME_MARGIN_M: f64 = 1.0;

/// 로봇 마운트 y [m] — 궤적을 그릴 하한.
///
/// 그 뒤는 로봇이 이미 지나친 자리라 볼 이유가 없고, 길게 그려 두면 화면만 어지럽다.
/// 예측은 여전히 그 너머까지 적분한다(바운스·평면 통과 계산에 필요하다) — **그리기만**
/// 여기서 자른다.
pub fn draw_limit_y() -> f64 {
    return defaults::rail_frame().mount_y();
}

/// `y >= draw_limit_y()` 구간만 남긴다. 경계를 지나는 선분은 그 지점에서 끊는다.
pub fn clip_to_mount(points: &[Point3]) -> Vec<Point3> {
    let limit = draw_limit_y();
    let mut out = Vec::with_capacity(points.len());
    for pair in points.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if a.y >= limit {
            out.push(a);
        }
        // 경계를 가로지르면 정확히 그 점까지만 그린다.
        if (a.y >= limit) != (b.y >= limit) {
            let span = a.y - b.y;
            if span.abs() > f64::EPSILON {
                out.push(a.lerp(&b, (a.y - limit) / span));
            }
            break;
        }
    }
    if let Some(last) = points.last()
        && last.y >= limit
        && points.len() > 1
    {
        out.push(*last);
    }
    if points.len() == 1 && points[0].y >= limit {
        out.push(points[0]);
    }
    return out;
}

/// `[x y z vx vy vz t]`. `t`는 클립 시작 기준 [s].
#[derive(Debug, Clone, Copy)]
pub struct State7 {
    pub t: f64,
    pub position: Point3,
    pub velocity: Vector3<f64>,
}

/// 한 프레임에서 실제로 관측된 것.
#[derive(Debug, Clone, Copy)]
pub struct Observed {
    pub frame: usize,
    pub t: f64,
    pub point: Point3,
    /// 복원한 점을 두 캠에 되쏜 최대 오차 [px].
    pub reprojection_px: f64,
}

/// 프레임 하나의 재생 상태 — 창이 그릴 것 전부.
#[derive(Debug, Clone, Default)]
pub struct FrameState {
    /// 캠별 검출 픽셀 (`None` = 못 찾음).
    pub pixels: [Option<camera::Pixel>; 2],
    /// 이 프레임의 삼각측량 (둘 다 잡고 재투영 게이트를 통과했을 때만).
    pub observed: Option<Observed>,
    /// 이 시점 EKF 상태에서 앞으로 굴린 궤적. 추적 중이 아니면 비어 있다.
    ///
    /// 매 프레임 현재 위치에서 다시 굴린 것이라 **실제 궤적에 붙어 보인다** — 당연하다.
    /// "예측이 맞았나"는 이걸로 판정하는 게 아니라 [`Commit`]으로 판정한다.
    pub predicted: Vec<State7>,
    /// 실기와 **같은 게이트**([`decide`])의 이번 프레임 판정.
    pub decision: Option<Decision>,
    /// `hypot(σ_p, σ_v × 리드)` — 커밋 게이트의 마지막 관문.
    pub impact_sigma: Option<f64>,
    pub ekf_position: Option<Point3>,
    pub ekf_speed: Option<f64>,
    pub tracking: bool,
    pub gate: Option<GateOutcome>,
    pub gate_d2: Option<f64>,
    /// `hypot(σ_p, σ_v × 리드)` 를 못 내는 상태면 `None`.
    pub position_sigma: Option<f64>,
    pub velocity_sigma: Option<f64>,
}

/// 실기가 하드웨어에 예측을 넘겼을 순간 — 그리고 그때 넘긴 것.
///
/// 실기 파이프라인은 [`decide`]가 `Attempt`를 낸 프레임에 `CommitRequest`를 보내고,
/// 제어 워커가 그걸로 궤적을 짜고 나면 스윙이 끝날 때까지 `Recovering`이라 새 요청을
/// 받지 않는다. 즉 **한 샷에 실질적으로 한 번**이다. 그 한 번을 여기 얼려 둔다.
///
/// 이게 얼려 있어야 "예측이 맞았나"를 볼 수 있다. 매 프레임 다시 굴린 예측은 언제나
/// 현재 공 위치에서 출발하니 실제와 겹쳐 보일 수밖에 없다 — 볼 게 없다.
#[derive(Debug, Clone)]
pub struct Commit {
    pub frame: usize,
    pub t: f64,
    /// 커밋 순간의 예측 궤적 — 이후 프레임에서도 **바뀌지 않는다**.
    pub predicted: Vec<State7>,
    /// 표시용 대표 후보 (커밋 창 안의 첫 후보, 없으면 tti가 가장 이른 것).
    ///
    /// 실기 플래너(`plan_best`)가 실제로 고르는 후보와 반드시 같지는 않다 — 그건 IK
    /// 점수로 고르는데 여기엔 팔이 없다. 어디를 칠 셈이었는지 가늠하는 표시다.
    pub impact: Point3,
    pub time_to_impact: f64,
    pub impact_sigma: f64,
}

/// 클립 전체 — 창이 필요로 하는 모든 것.
pub struct Reviewed {
    pub frames: Vec<FrameState>,
    /// 클립 전체의 실제 궤적 (재생 위치와 무관하게 이미 다 안다).
    pub observed: Vec<Observed>,
    /// 첫 `Attempt` — 실기라면 여기서 하드웨어로 넘어갔다.
    pub commit: Option<Commit>,
    pub fps: f64,
}

impl Reviewed {
    pub fn len(&self) -> usize {
        return self.frames.len();
    }

    /// 프레임 인덱스 → 클립 시작 기준 시각 [s].
    pub fn time_of(&self, frame: usize) -> f64 {
        return frame as f64 / self.fps;
    }

    /// 실제 궤적을 현재 프레임 기준 (지금까지, 이후)로 나눈다.
    ///
    /// 오프라인 재생이라 미래를 **이미 안다** — 커밋 예측이 이후로 어디로 갔는지와 나란히
    /// 보려면 있어야 한다. 다만 과거와 같은 굵기로 그리면 "지금 아는 것"과 구분이 안 되므로
    /// 나눠서 넘긴다. 경계 한 점은 양쪽에 다 들어간다 (선이 끊겨 보이지 않게).
    pub fn observed_split(&self, frame: usize) -> (Vec<Point3>, Vec<Point3>) {
        let cut = self.observed.partition_point(|o| o.frame <= frame);
        let past: Vec<Point3> = self.observed[..cut].iter().map(|o| o.point).collect();
        let future: Vec<Point3> = self.observed[cut.saturating_sub(1)..]
            .iter()
            .map(|o| o.point)
            .collect();
        return (past, future);
    }

    /// 실제 궤적이 `y` 평면을 로봇 쪽으로 지난 지점 (표본 사이 선형 보간).
    ///
    /// 커밋 예측의 타점과 비교할 **정답**이다.
    pub fn observed_crossing_y(&self, y: f64) -> Option<Point3> {
        let pair = self
            .observed
            .windows(2)
            .find(|w| w[0].point.y >= y && w[1].point.y < y)?;
        let (a, b) = (pair[0].point, pair[1].point);
        let span = a.y - b.y;
        if span <= f64::EPSILON {
            return Some(b);
        }
        return Some(a.lerp(&b, (a.y - y) / span));
    }
}

/// 클립을 한 번 훑어 검출·삼각측량·EKF 재생을 끝낸다.
pub fn review(left: &Path, right: &Path, fps: f64) -> Result<Reviewed, String> {
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(|e| format!("calibration 로드: {e}"))?;

    let left_pixels = detect_all(left, camera::Id(0))?;
    let right_pixels = detect_all(right, camera::Id(1))?;
    let count = left_pixels.len().min(right_pixels.len());

    let physics = PhysicsParams::default();
    let planes = InterceptWindow::default().hit_planes();
    let epoch = Instant::now();
    let mut ekf = Ekf::default();
    let mut frames: Vec<FrameState> = Vec::with_capacity(count);
    let mut observed: Vec<Observed> = Vec::new();
    let mut commit: Option<Commit> = None;

    for frame in 0..count {
        let t = frame as f64 / fps;
        let pixels = [left_pixels[frame], right_pixels[frame]];
        let hit = triangulate(&pixels, &calibration).map(|(point, reprojection_px)| Observed {
            frame,
            t,
            point,
            reprojection_px,
        });

        let mut state = FrameState {
            pixels,
            observed: hit,
            ..FrameState::default()
        };

        if let Some(hit) = hit {
            observed.push(hit);
            state.gate = Some(ekf.update_position(hit.point, epoch + secs(t)));
            state.gate_d2 = ekf.last_gate_d2();
        }

        state.tracking = ekf.is_tracking();
        state.ekf_position = ekf.position();
        state.ekf_speed = ekf.velocity().map(|v| v.norm());
        state.position_sigma = ekf.position_sigma();
        state.velocity_sigma = ekf.velocity_sigma();
        if let (Some(position), Some(velocity)) = (ekf.position(), ekf.velocity()) {
            state.predicted = rollout(position, velocity, t, &physics);
        }

        // 실기와 같은 게이트를 그대로 태운다 — 커밋 시점이 실기와 달라지면 진단이 거짓말한다.
        let predictions: Vec<Prediction> = if state.tracking {
            planes
                .iter()
                .filter_map(|plane| ekf.predict_to(*plane))
                .collect()
        } else {
            Vec::new()
        };
        let shown = display_candidate(&predictions);
        state.impact_sigma = shown.and_then(|prediction| {
            let (sp, sv) = (ekf.position_sigma()?, ekf.velocity_sigma()?);
            Some(sp.hypot(sv * prediction.time_to_impact_secs))
        });
        let ball_y = state.ekf_position.map(|p| p.y);
        let decision = decide(state.tracking, ball_y, &predictions, state.impact_sigma);
        state.decision = Some(decision);

        // 첫 Attempt만 잡는다. 실기도 커밋 뒤에는 Recovering이라 한 샷에 한 번이다.
        if commit.is_none()
            && decision == Decision::Attempt
            && let (Some(prediction), Some(sigma)) = (shown, state.impact_sigma)
        {
            commit = Some(Commit {
                frame,
                t,
                predicted: state.predicted.clone(),
                impact: prediction.impact_position,
                time_to_impact: prediction.time_to_impact_secs,
                impact_sigma: sigma,
            });
        }
        frames.push(state);
    }

    return Ok(Reviewed {
        frames,
        observed,
        commit,
        fps,
    });
}

/// 화면에 대표로 보여줄 후보 — 커밋 창 안의 첫 후보, 없으면 tti가 가장 이른 것.
/// (`src/real/estimator_worker.rs`의 같은 이름 함수와 동일한 규칙.)
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

fn secs(t: f64) -> Duration {
    return Duration::from_secs_f64(t.max(0.0));
}

/// 프레임별 검출 픽셀. 본선과 **같은 조립**(`detector_for`)을 쓴다.
fn detect_all(path: &Path, camera_id: camera::Id) -> Result<Vec<Option<camera::Pixel>>, String> {
    let mut source = OpenCvCapture::from_path(camera_id, path)?;
    let mut detector = defaults::detector_for(camera_id).map_err(|e| format!("detector: {e}"))?;
    let mut out = Vec::new();
    while let Some(frame) = source.next_frame() {
        out.push(detector.detect(&frame));
    }
    return Ok(out);
}

/// 두 캠이 같은 프레임에서 잡은 것만 복원한다. 재투영이 벌어지면 서로 다른 걸 잡은 것.
fn triangulate(
    pixels: &[Option<camera::Pixel>; 2],
    calibration: &Calibration,
) -> Option<(Point3, f64)> {
    let (Some(left), Some(right)) = (pixels[0], pixels[1]) else {
        return None;
    };
    let hits = [(camera::Id(0), left), (camera::Id(1), right)];
    let point = Triangulate::pixels(&hits, calibration)?;
    let worst = hits
        .iter()
        .filter_map(|(id, pixel)| {
            let params = calibration.params(*id)?;
            let projected = params.project_world_unclipped(point)?;
            Some((projected - *pixel).norm())
        })
        .fold(0.0_f64, f64::max);
    if worst > MAX_REPROJECTION_PX {
        return None;
    }
    return Some((point, worst));
}

/// 현재 상태에서 공이 플레이 부피를 벗어날 때까지 굴린다.
///
/// 잘라 주지 않는다 — 접수 평면 하나가 아니라 궤적 전체다. 바운스는
/// [`Kinematics::step`]이 커널 SSOT로 처리하므로 여기서 따로 볼 게 없다.
fn rollout(
    position: Point3,
    velocity: Vector3<f64>,
    t0: f64,
    physics: &PhysicsParams,
) -> Vec<State7> {
    let mut p = position.coords;
    let mut v = velocity;
    let omega = Vector3::zeros();
    let mut out = vec![State7 {
        t: t0,
        position,
        velocity,
    }];
    let mut elapsed = 0.0_f64;
    let mut since_sample = 0.0_f64;

    while elapsed < HORIZON_SECS {
        let (next_p, next_v, _) = Kinematics::step(p, v, omega, INTEGRATE_DT, physics);
        p = next_p;
        v = next_v;
        elapsed += INTEGRATE_DT;
        since_sample += INTEGRATE_DT;
        if since_sample >= SAMPLE_DT {
            since_sample = 0.0;
            out.push(State7 {
                t: t0 + elapsed,
                position: Point3::from(p),
                velocity: v,
            });
        }
        if outside_play_volume(p) {
            break;
        }
    }
    return out;
}

fn outside_play_volume(p: Vector3<f64>) -> bool {
    return p.y < -VOLUME_MARGIN_M
        || p.y > table::LENGTH_Y + VOLUME_MARGIN_M
        || p.x < -VOLUME_MARGIN_M
        || p.x > table::WIDTH_X + VOLUME_MARGIN_M;
}

/// `t`에서의 예측 위치 (표본 사이는 선형 보간). 궤적 밖이면 `None`.
pub fn predicted_at(predicted: &[State7], t: f64) -> Option<Point3> {
    let first = predicted.first()?;
    let last = predicted.last()?;
    if t < first.t || t > last.t {
        return None;
    }
    let index = predicted.partition_point(|s| s.t <= t);
    if index == 0 {
        return Some(first.position);
    }
    let Some(later) = predicted.get(index) else {
        return Some(last.position);
    };
    let earlier = predicted[index - 1];
    let span = later.t - earlier.t;
    if span <= f64::EPSILON {
        return Some(earlier.position);
    }
    let w = (t - earlier.t) / span;
    return Some(earlier.position.lerp(&later.position, w));
}

/// 예측 궤적이 `y` 평면을 로봇 쪽으로 지나는 첫 상태.
///
/// 제어측이 실제로 쓰는 숫자다 — 어디를, 얼마의 속도로 맞을지. 궤적을 통째로 넘기면
/// 이런 질문은 전부 소비자 쪽 한 줄이 된다 (비전이 접수 평면을 알 필요가 없다).
pub fn crossing_y(predicted: &[State7], y: f64) -> Option<State7> {
    return predicted
        .windows(2)
        .find(|w| w[0].position.y >= y && w[1].position.y < y)
        .map(|w| w[1]);
}

/// 리드타임 `lead` 뒤의 **예측**과 그때의 **실제**가 얼마나 벌어지는가 [m].
///
/// 이게 이 툴의 본론이다. 실제 궤적은 클립 전체를 이미 훑어서 알고 있으므로,
/// 재생 중 어느 프레임에서든 "그때 한 예측이 맞았는지"를 바로 잴 수 있다.
pub fn convergence_error(
    predicted: &[State7],
    observed: &[Observed],
    now: f64,
    lead: f64,
    fps: f64,
) -> Option<f64> {
    let target = now + lead;
    // 프레임 반 칸 안쪽의 실제 관측만 짝으로 인정한다 — 없으면 잴 수 없다.
    let tolerance = 0.5 / fps;
    let truth = observed
        .iter()
        .filter(|o| (o.t - target).abs() <= tolerance)
        .min_by(|a, b| {
            (a.t - target)
                .abs()
                .partial_cmp(&(b.t - target).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
    let guess = predicted_at(predicted, truth.t)?;
    return Some((guess - truth.point).norm());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// y가 1 m/s로 줄어드는 직선 궤적, 0.1 s 간격.
    fn straight() -> Vec<State7> {
        return (0..=10)
            .map(|i| {
                let t = f64::from(i) * 0.1;
                State7 {
                    t,
                    position: Point3::new(0.0, 1.0 - t, 1.0),
                    velocity: Vector3::new(0.0, -1.0, 0.0),
                }
            })
            .collect();
    }

    #[test]
    fn predicted_at_interpolates_between_samples() {
        let track = straight();
        let mid = predicted_at(&track, 0.15).expect("궤적 안");
        assert!((mid.y - 0.85).abs() < 1e-9, "y={}", mid.y);
    }

    #[test]
    fn predicted_at_refuses_outside_the_track() {
        let track = straight();
        assert!(predicted_at(&track, -0.1).is_none(), "시작 전");
        assert!(predicted_at(&track, 1.5).is_none(), "끝 이후");
    }

    #[test]
    fn convergence_error_measures_the_gap_to_truth() {
        let track = straight();
        // 실제는 예측보다 5 cm 옆으로 갔다.
        let observed = vec![Observed {
            frame: 20,
            t: 0.2,
            point: Point3::new(0.05, 0.8, 1.0),
            reprojection_px: 1.0,
        }];
        let error = convergence_error(&track, &observed, 0.0, 0.2, 100.0).expect("짝 성립");
        assert!((error - 0.05).abs() < 1e-9, "error={error}");
    }

    /// 짝이 없으면 **0이 아니라 없음**이어야 한다 — 못 잰 걸 잘 맞춘 걸로 읽으면 안 된다.
    #[test]
    fn convergence_error_is_none_without_a_matching_observation() {
        let track = straight();
        let observed = vec![Observed {
            frame: 90,
            t: 0.9,
            point: Point3::new(0.0, 0.1, 1.0),
            reprojection_px: 1.0,
        }];
        assert!(convergence_error(&track, &observed, 0.0, 0.2, 100.0).is_none());
    }

    fn at_y(y: f64) -> Point3 {
        return Point3::new(0.5, y, 1.0);
    }

    #[test]
    fn clip_keeps_a_track_that_never_reaches_the_mount() {
        let limit = draw_limit_y();
        let track = vec![at_y(limit + 1.0), at_y(limit + 0.5), at_y(limit + 0.2)];
        assert_eq!(clip_to_mount(&track).len(), 3);
    }

    #[test]
    fn clip_ends_exactly_at_the_mount() {
        let limit = draw_limit_y();
        let track = vec![at_y(limit + 0.4), at_y(limit + 0.2), at_y(limit - 0.3)];
        let clipped = clip_to_mount(&track);
        let last = clipped.last().expect("경계점이 남는다");
        assert!((last.y - limit).abs() < 1e-9, "y={} != {limit}", last.y);
        // 마운트 뒤 점은 하나도 안 남는다.
        assert!(clipped.iter().all(|p| p.y >= limit - 1e-9));
    }

    #[test]
    fn clip_drops_a_track_entirely_behind_the_mount() {
        let limit = draw_limit_y();
        let track = vec![at_y(limit - 0.1), at_y(limit - 0.5)];
        assert!(clip_to_mount(&track).len() < 2, "그릴 선분이 없어야 한다");
    }

    #[test]
    fn crossing_y_reports_position_and_velocity_at_the_plane() {
        let track = straight();
        let hit = crossing_y(&track, 0.5).expect("평면을 지난다");
        assert!(
            hit.position.y < 0.5 && hit.position.y > 0.35,
            "y={}",
            hit.position.y
        );
        assert!((hit.velocity.y + 1.0).abs() < 1e-9);
        assert!(crossing_y(&track, -5.0).is_none(), "안 지나는 평면");
    }
}
