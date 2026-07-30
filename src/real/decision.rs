//! 스윙 커밋 게이트 — 하드웨어에 손대지 않는 순수 판정.
//!
//! sim [`SimWorld::try_auto_swing`]의 게이트 **순서를 그대로** 옮겼다. 상수는 전부
//! `defaults::ControlParams` / `robot::motion::Planner`에서 오고, real 전용 값을 새로 만들지
//! 않는다 — sim과 real이 갈리면 sim에서 튜닝한 결과가 실기에 안 옮겨진다.
//!
//! **여기서 포기는 하지 않는다.** 남은 시간이 짧다는 것만으로 공을 버리지 않고, 실현
//! 불가능한지는 `Planner::plan_best`의 속도·가속·토크·관절범위 검사가 판정한다.
//!
//! 하드웨어에 손대지 않는 순수 함수라 `cargo test --workspace`로 그대로 돈다.
//!
//! [`SimWorld::try_auto_swing`]: https://github.com/luv-all/pingpong-bot/blob/main/src/sim/physics/world.rs

use pingpong_bot::defaults::EstimatorParams;
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
    /// 커밋 창(`(0, swing_commit_max_secs]`) 밖 — 아직 너무 이르다.
    OutOfWindow,
    /// 예측 도달점 불확실성이 아직 크다 — 필터가 확신하지 못한다.
    Uncertain,
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
            Self::Uncertain => "WAIT uncertain prediction",
        };
    }
}

/// 이번 틱에 무엇을 할지.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    /// 다음 관측을 기다린다.
    Wait(WaitReason),
    /// 제어 워커에 계획을 요청한다.
    Attempt,
}

/// 게이트를 순서대로 통과시킨다.
///
/// `ball_y`는 EKF가 추정한 현재 공 y [m]. `None`이면 아직 추적 전으로 본다.
///
/// `impact_sigma`는 필터가 스스로 말하는 도달점 불확실성 [m]
/// (`hypot(σ_p, σ_v × 리드타임)`). 속도는 측정되지 않고 위치 차분에서 나오므로 시드 직후엔
/// 미터 단위로 틀린다 — 확신이 설 때까지 기다린다. `None`이면 아직 못 잰 것으로 보고 기다린다.
pub fn decide(
    tracking: bool,
    ball_y: Option<f64>,
    predictions: &[Prediction],
    impact_sigma: Option<f64>,
) -> Decision {
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

    // "너무 늦어서 포기"는 없다 (2026-07-31). 촉박한 것 자체는 위험하지 않고, 위험한 건
    // 그 시간에 맞추려고 요구되는 관절 속도·가속·토크다 — 그건 `Planner::plan_best`가
    // `kinematic_limit_violation`·`peak_torque_utilization`으로 각각 거부한다.
    // 포기는 그 한계에 실제로 걸렸을 때(`JointOrTorqueLimit`)만 제어 워커가 판정한다.
    if !predictions
        .iter()
        .any(|prediction| Planner::in_commit_window(prediction.time_to_impact_secs))
    {
        return Decision::Wait(WaitReason::OutOfWindow);
    }

    // 마지막 관문 — 필터가 확신할 때만 친다. 시드 직후 예측은 미터 단위로 틀리고,
    // 커밋은 되돌릴 수 없다.
    let limit = EstimatorParams::default().max_impact_sigma;
    if !impact_sigma.is_some_and(|sigma| sigma <= limit) {
        return Decision::Wait(WaitReason::Uncertain);
    }
    return Decision::Attempt;
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::Vector3;
    use pingpong_bot::Point3;
    use pingpong_bot::constants::table;
    use pingpong_bot::defaults::ControlParams;

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

    /// 게이트를 통과하는 확신도 — 한계보다 넉넉히 작다.
    fn confident() -> Option<f64> {
        return Some(EstimatorParams::default().max_impact_sigma * 0.1);
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
            confident(),
        );
        assert_eq!(decision, Decision::Wait(WaitReason::NoTrack));
    }

    #[test]
    fn waits_when_no_hit_plane_yields_a_prediction() {
        let decision = decide(true, Some(past_midcourt_y()), &[], confident());
        assert_eq!(decision, Decision::Wait(WaitReason::NoPrediction));
    }

    #[test]
    fn waits_until_the_ball_is_past_midcourt() {
        let decision = decide(
            true,
            Some(before_midcourt_y()),
            &[prediction(in_window_secs())],
            confident(),
        );
        assert_eq!(decision, Decision::Wait(WaitReason::BeforeMidcourt));
    }

    /// 시간이 촉박해도 **포기하지 않고 시도한다** (2026-07-31 결정).
    ///
    /// 예전에는 모든 후보의 tti가 `min_swing_secs`(0.20 s) 미만이면 공을 통째로 버렸다.
    /// 촉박한 것 자체는 위험하지 않다 — 위험한 건 그 시간에 요구되는 속도·토크이고 그건
    /// 플래너가 따로 거부한다. 시간 하한을 다시 넣으면 이 테스트가 깨진다.
    #[test]
    fn attempts_even_when_every_candidate_is_late() {
        let very_late = ControlParams::default().min_swing_secs * 0.1;
        assert!(
            Planner::in_commit_window(very_late),
            "커밋 창에 시간 하한이 없어야 한다"
        );

        let decision = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(very_late), prediction(very_late * 0.5)],
            confident(),
        );
        assert_eq!(
            decision,
            Decision::Attempt,
            "늦어도 시도한다 — 실현 가능 여부는 플래너의 물리 한계가 판정한다"
        );
    }

    /// 수치 하한(0에 수렴하는 tti)만은 여전히 막는다 — quintic이 0으로 나눈다.
    #[test]
    fn rejects_only_a_numerically_degenerate_time_to_go() {
        let degenerate = pingpong_bot::defaults::MIN_TIME_TO_GO_SECS * 0.5;
        assert!(!Planner::in_commit_window(degenerate));

        let decision = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(degenerate)],
            confident(),
        );
        assert_eq!(decision, Decision::Wait(WaitReason::OutOfWindow));
    }

    #[test]
    fn waits_while_every_candidate_is_outside_the_commit_window() {
        let too_early = ControlParams::default().swing_commit_max_secs * 2.0;
        assert!(!Planner::in_commit_window(too_early));

        let decision = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(too_early)],
            confident(),
        );
        assert_eq!(decision, Decision::Wait(WaitReason::OutOfWindow));
    }

    /// 필터가 확신하지 못하면 커밋하지 않는다.
    ///
    /// 속도는 측정되지 않고 위치 차분에서 나와 시드 직후 σ_v가 1~2 m/s다 — 그 상태의
    /// 도달점 예측은 미터 단위로 틀린다 (fly_04 실측 245 cm). 커밋은 되돌릴 수 없으므로
    /// 필터가 스스로 좁혔다고 말할 때까지 기다린다.
    #[test]
    fn waits_while_the_predicted_impact_is_uncertain() {
        let limit = EstimatorParams::default().max_impact_sigma;

        let uncertain = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(in_window_secs())],
            Some(limit * 2.0),
        );
        assert_eq!(uncertain, Decision::Wait(WaitReason::Uncertain));

        // 아직 재지 못한 경우도 기다린다 — 모르면 치지 않는다.
        let unknown = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(in_window_secs())],
            None,
        );
        assert_eq!(unknown, Decision::Wait(WaitReason::Uncertain));
    }

    #[test]
    fn attempts_once_a_candidate_is_inside_the_commit_window() {
        let decision = decide(
            true,
            Some(past_midcourt_y()),
            &[prediction(in_window_secs())],
            confident(),
        );
        assert_eq!(decision, Decision::Attempt);
    }
}
