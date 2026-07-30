//! 스윙 커밋 게이트 — 하드웨어에 손대지 않는 순수 판정.
//!
//! sim [`SimWorld::try_auto_swing`]의 게이트 **순서를 그대로** 옮겼다. 상수는 전부
//! `defaults::ControlParams` / `robot::motion::Planner`에서 오고, real 전용 값을 새로 만들지
//! 않는다 — sim과 real이 갈리면 sim에서 튜닝한 결과가 실기에 안 옮겨진다.
//!
//! 하드웨어에 손대지 않는 순수 함수라 `cargo test --workspace`로 그대로 돈다.
//!
//! [`SimWorld::try_auto_swing`]: https://github.com/luv-all/pingpong-bot/blob/main/src/sim/physics/world.rs

use pingpong_bot::defaults::ControlParams;
use pingpong_bot::estimator::Prediction;
use pingpong_bot::robot::motion::Planner;

/// 아직 칠 때가 아닌 이유 — HUD·로그용.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitReason {
    /// EKF가 아직 속도까지 못 잡음.
    NoTrack,
    /// 접수 평면 교차 예측이 하나도 없음 (네트 게이트 포함).
    NoPrediction,
    /// 공이 아직 미드코트를 안 넘음.
    BeforeMidcourt,
    /// 커밋 창(`[min_swing_secs, swing_commit_max_secs]`) 밖.
    OutOfWindow,
}

impl WaitReason {
    /// 프리뷰 HUD 라벨. **ASCII만** — OpenCV Hershey 폰트는 유니코드를 못 그려서 한글을 넣으면
    /// `??????`로 나온다 (`camera::Preview::draw_debug_lines`).
    pub fn label(self) -> &'static str {
        return match self {
            Self::NoTrack => "WAIT no track",
            Self::NoPrediction => "WAIT no prediction",
            Self::BeforeMidcourt => "WAIT before midcourt",
            Self::OutOfWindow => "WAIT out of commit window",
        };
    }
}

/// 이번 틱에 무엇을 할지.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    /// 다음 관측을 기다린다.
    Wait(WaitReason),
    /// 이 공은 포기한다 (단발이므로 곧 종료).
    Abandon(&'static str),
    /// 제어 워커에 계획을 요청한다.
    Attempt,
}

/// 후보 중 **가장 늦게** 도달하는 tti [s]. 후보가 없으면 `None`.
///
/// "너무 늦음" 판정의 근거값이다 — `min`이 아니라 `max`인 이유는 [`decide`] 참고.
/// 로그에 실을 때도 같은 값을 쓰도록 여기서 한 번만 계산한다.
pub fn latest_tti_secs(predictions: &[Prediction]) -> Option<f64> {
    return predictions
        .iter()
        .map(|prediction| prediction.time_to_impact_secs)
        .reduce(f64::max);
}

/// 게이트를 순서대로 통과시킨다.
///
/// `ball_y`는 EKF가 추정한 현재 공 y [m]. `None`이면 아직 추적 전으로 본다.
pub fn decide(tracking: bool, ball_y: Option<f64>, predictions: &[Prediction]) -> Decision {
    if !tracking {
        return Decision::Wait(WaitReason::NoTrack);
    }
    if predictions.is_empty() {
        return Decision::Wait(WaitReason::NoPrediction);
    }
    let Some(ball_y) = ball_y else {
        return Decision::Wait(WaitReason::NoTrack);
    };
    if !Planner::past_midcourt(ball_y) {
        return Decision::Wait(WaitReason::BeforeMidcourt);
    }

    // `max`다 — `min`으로 쓰면 아직 여유 있는 후보가 하나라도 늦은 후보에 끌려가 통째로
    // 포기된다. sim에서 이 실수로 커밋률이 0%가 된 이력이 world.rs 주석에 남아 있다.
    let latest = predictions
        .iter()
        .map(|prediction| prediction.time_to_impact_secs)
        .fold(f64::NEG_INFINITY, f64::max);
    if latest < ControlParams::default().min_swing_secs {
        return Decision::Abandon("너무 늦음 — 남은 시간이 최소 스윙 시간 미만");
    }

    if !predictions
        .iter()
        .any(|prediction| Planner::in_commit_window(prediction.time_to_impact_secs))
    {
        return Decision::Wait(WaitReason::OutOfWindow);
    }
    return Decision::Attempt;
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::Vector3;
    use pingpong_bot::Point3;
    use pingpong_bot::constants::table;

    /// 미드코트를 넘은 (= 커밋 게이트를 통과하는) 공 y.
    fn past_midcourt_y() -> f64 {
        let y = table::LENGTH_Y * ControlParams::default().swing_commit_max_ball_y_frac * 0.5;
        assert!(
            Planner::past_midcourt(y),
            "테스트 픽스처가 게이트를 통과해야 함"
        );
        return y;
    }

    /// 아직 미드코트 전인 공 y.
    fn before_midcourt_y() -> f64 {
        let y = table::LENGTH_Y;
        assert!(!Planner::past_midcourt(y));
        return y;
    }

    fn prediction(time_to_impact_secs: f64) -> Prediction {
        return Prediction {
            time_to_impact_secs,
            impact_position: Point3::new(0.7, 0.2, table::SURFACE_Z + 0.2),
            incoming_velocity: Vector3::new(0.0, -4.0, 0.0),
        };
    }

    /// 커밋 창 한가운데 tti.
    fn in_window_secs() -> f64 {
        let control = ControlParams::default();
        let t = (control.min_swing_secs + control.swing_commit_max_secs) * 0.5;
        assert!(Planner::in_commit_window(t));
        return t;
    }

    #[test]
    fn waits_until_the_filter_has_a_velocity() {
        let decision = decide(
            false,
            Some(past_midcourt_y()),
            &[prediction(in_window_secs())],
        );
        assert_eq!(decision, Decision::Wait(WaitReason::NoTrack));
    }

    #[test]
    fn waits_when_no_hit_plane_yields_a_prediction() {
        let decision = decide(true, Some(past_midcourt_y()), &[]);
        assert_eq!(decision, Decision::Wait(WaitReason::NoPrediction));
    }

    #[test]
    fn waits_until_the_ball_is_past_midcourt() {
        let decision = decide(
            true,
            Some(before_midcourt_y()),
            &[prediction(in_window_secs())],
        );
        assert_eq!(decision, Decision::Wait(WaitReason::BeforeMidcourt));
    }

    /// 너무 늦음 판정은 `max(tti)` 기준이다.
    ///
    /// 늦은 후보 하나가 섞여 있어도, 아직 여유 있는 후보가 남아 있으면 포기하면 안 된다.
    /// `min`으로 바꾸면 이 테스트가 깨진다 (sim 커밋률 0% 회귀 방지).
    #[test]
    fn abandons_only_when_every_candidate_is_too_late() {
        let control = ControlParams::default();
        let too_late = control.min_swing_secs * 0.5;

        let all_late = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(too_late), prediction(too_late * 0.5)],
        );
        assert!(
            matches!(all_late, Decision::Abandon(_)),
            "전부 늦으면 포기: {all_late:?}"
        );

        let mixed = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(too_late), prediction(in_window_secs())],
        );
        assert_eq!(
            mixed,
            Decision::Attempt,
            "여유 있는 후보가 남아 있으면 포기하지 않는다 (max이지 min이 아니다)"
        );
    }

    #[test]
    fn waits_while_every_candidate_is_outside_the_commit_window() {
        let too_early = ControlParams::default().swing_commit_max_secs * 2.0;
        assert!(!Planner::in_commit_window(too_early));

        let decision = decide(true, Some(past_midcourt_y()), &[prediction(too_early)]);
        assert_eq!(decision, Decision::Wait(WaitReason::OutOfWindow));
    }

    #[test]
    fn attempts_once_a_candidate_is_inside_the_commit_window() {
        let decision = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(in_window_secs())],
        );
        assert_eq!(decision, Decision::Attempt);
    }
}
