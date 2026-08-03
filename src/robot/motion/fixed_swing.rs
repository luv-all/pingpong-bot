//! 사전 정의된 스윙 딕셔너리 — IK 없이 시작/끝 관절각만으로 스윙한다.
//!
//! `Planner::plan_best_swing`/`robot::control::PositionController`와 달리 라켓
//! 위치·자세를 IK로 풀지 않는다. 스윙 "모양"은 [`FIXED_SWING_START_DEG`] →
//! [`FIXED_SWING_END_DEG`]로 고정이고, 호출부는 레일 x(기하만, IK 없음)와
//! 스윙을 시작할 타이밍만 정한다.

use crate::defaults;
use crate::error::DomainError;
use crate::robot::{Arm, Joints, LinearRail, Pose};

use super::{Planner, Trajectory};

/// 스윙 시작(백스윙/준비) 자세 [deg] — j0 yaw, j1 shoulder, j2 elbow, j3 wrist.
pub const FIXED_SWING_START_DEG: [f64; 4] = [-10.0, 0.0, 50.0, -30.0];
/// 스윙 끝(임팩트) 자세 [deg] — 관절 순서는 시작과 동일.
pub const FIXED_SWING_END_DEG: [f64; 4] = [40.0, 0.0, -12.0, -70.0];

pub fn fixed_swing_start_joints() -> Joints {
    return Joints::from_slice(&FIXED_SWING_START_DEG.map(f64::to_radians));
}

pub fn fixed_swing_end_joints() -> Joints {
    return Joints::from_slice(&FIXED_SWING_END_DEG.map(f64::to_radians));
}

/// 레일 `rail_x`에 고정한 채, IK 없이 시작→끝 관절각을 모터 한계(속도·가속·
/// 토크) 100%로 잇는 가장 빠른 quintic. `should_start_fixed_swing`이 이
/// 결과의 `duration_secs`를 스윙 시작 타이밍 판정에 쓴다.
pub fn plan_fixed_swing(arm: &Arm, rail_x: f64) -> Result<Trajectory, DomainError> {
    let start = Pose::new(rail_x, fixed_swing_start_joints());
    return Planner::move_to_fastest(arm, &start, fixed_swing_end_joints(), rail_x);
}

/// 예측 임팩트 x를 레일 사거리 안으로 자른다 — IK 없이 기하만으로 리니어
/// 목표를 정한다.
pub fn fixed_swing_rail_target(rail: &LinearRail, predicted_impact_x: f64) -> f64 {
    return rail.clamp_x(predicted_impact_x);
}

/// 고정 스윙 내부에서 라켓이 공과 만난다고 가정하는 시각을 고르는 전략.
///
/// 라켓은 START→END를 실시간으로 스윕하므로, 공은 그 스윕 **도중** 만나야
/// 한다 — 스윙이 끝나는 순간(= END 자세 도달)을 임팩트로 보면 안 된다
/// (2026-08-03 실측 회귀: 기본 슈터 샷에서 스윙 전체 소요 시간이 로봇
/// 접수창의 어떤 평면에 대해서도 남은 비행 시간보다 길어, "지금 시작해도
/// 이미 늦음"이 발사 첫 틱부터 참이었다).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImpactTimeStrategy {
    /// 스윙 소요 시간의 정확히 절반.
    Midpoint,
    /// 라켓 중심의 순간 속력(FK 유한차분)이 최대가 되는 시각.
    PeakRacketSpeed,
}

/// 두 전략을 비교 중이라 기본값은 더 단순하고 예측 가능한 쪽으로 둔다.
pub const DEFAULT_IMPACT_TIME_STRATEGY: ImpactTimeStrategy = ImpactTimeStrategy::Midpoint;

/// `strategy`에 따라 고정 스윙 내부의 가정 임팩트 시각 [s]을 고른다
/// (`0 < 반환값 < trajectory.duration_secs`).
///
/// 레일은 스윙 도중 고정이라(`plan_fixed_swing`이 `rail_x`를 시작=끝으로 둔다)
/// 라켓의 시간에 따른 **속도** 형태는 `rail_x`와 무관하다 — 위치만
/// 평행이동한다. 그래서 이 계산은 궤적당 한 번만 하면 되고, 레일 위치가
/// 바뀌어도 다시 스윕할 필요가 없다(다만 인터페이스는 `rail_x`를 그대로
/// 받아 FK 위치를 실제로 구한다 — 속도만 rail_x 불변이라는 뜻).
pub fn fixed_swing_impact_time_secs(
    arm: &Arm,
    rail_x: f64,
    trajectory: &Trajectory,
    strategy: ImpactTimeStrategy,
) -> f64 {
    return match strategy {
        ImpactTimeStrategy::Midpoint => trajectory.duration_secs * 0.5,
        ImpactTimeStrategy::PeakRacketSpeed => {
            peak_racket_speed_time(arm, rail_x, trajectory)
        }
    };
}

/// 라켓 중심 속력이 최대인 시각을 유한차분으로 찾는다 — 균등 표본
/// 중심차분, 표본 수는 정확도와 계산량의 실용적 절충.
fn peak_racket_speed_time(arm: &Arm, rail_x: f64, trajectory: &Trajectory) -> f64 {
    const SAMPLES: usize = 64;
    let duration = trajectory.duration_secs;
    if duration <= 0.0 {
        return 0.0;
    }
    let step = duration / SAMPLES as f64;
    let position_at = |t: f64| -> Option<nalgebra::Vector3<f64>> {
        return arm
            .forward_kinematics_with_rail(rail_x, &trajectory.sample_at(t))
            .map(|pose| pose.position.coords);
    };
    let mut best_time = duration * 0.5;
    let mut best_speed = -1.0_f64;
    for index in 0..=SAMPLES {
        let t = step * index as f64;
        let before = (t - step * 0.5).max(0.0);
        let after = (t + step * 0.5).min(duration);
        let span = (after - before).max(1e-9);
        if let (Some(p0), Some(p1)) = (position_at(before), position_at(after)) {
            let speed = (p1 - p0).norm() / span;
            if speed > best_speed {
                best_speed = speed;
                best_time = t;
            }
        }
    }
    return best_time;
}

/// 남은 시간이 스윙 **내부의 가정 임팩트 시각**([`fixed_swing_impact_time_secs`]) 이하가
/// 되는 즉시 스윙을 시작해야 한다 — 스윙 전체 소요 시간이 아니다. 스윙은
/// START→END를 실시간으로 스윕하므로, 공은 그 스윕 도중 만나야 한다.
pub fn should_start_fixed_swing(time_to_impact_secs: f64, impact_time_secs: f64) -> bool {
    return time_to_impact_secs.is_finite()
        && time_to_impact_secs > defaults::MIN_TIME_TO_GO_SECS
        && time_to_impact_secs <= impact_time_secs;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dictionary_joints_convert_degrees_to_radians() {
        let start = fixed_swing_start_joints();
        let end = fixed_swing_end_joints();
        for (actual, expected_deg) in start.values.iter().zip(FIXED_SWING_START_DEG) {
            assert!((actual.to_degrees() - expected_deg).abs() < 1e-9);
        }
        for (actual, expected_deg) in end.values.iter().zip(FIXED_SWING_END_DEG) {
            assert!((actual.to_degrees() - expected_deg).abs() < 1e-9);
        }
    }

    #[test]
    fn plan_fixed_swing_starts_and_ends_at_the_dictionary_poses_with_rail_held() {
        let robot = crate::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail");
        let rail_x = rail.default_x();

        let trajectory = plan_fixed_swing(&robot.arm, rail_x).expect("fixed swing plan");

        for (actual, expected) in trajectory
            .start
            .values
            .iter()
            .zip(fixed_swing_start_joints().values)
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        for (actual, expected) in trajectory
            .goal_joints()
            .values
            .iter()
            .zip(fixed_swing_end_joints().values)
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        assert!((trajectory.rail.start - rail_x).abs() < 1e-12);
        assert!((trajectory.rail.end - rail_x).abs() < 1e-12);
        assert!(trajectory.duration_secs > 0.0);
    }

    #[test]
    fn fixed_swing_rail_target_clamps_to_rail_range() {
        let robot = crate::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail");

        assert!((fixed_swing_rail_target(&rail, rail.x_min - 1.0) - rail.x_min).abs() < 1e-12);
        assert!((fixed_swing_rail_target(&rail, rail.x_max + 1.0) - rail.x_max).abs() < 1e-12);
        let mid = (rail.x_min + rail.x_max) * 0.5;
        assert!((fixed_swing_rail_target(&rail, mid) - mid).abs() < 1e-12);
    }

    #[test]
    fn should_start_fixed_swing_fires_only_inside_the_duration_window() {
        let swing_duration = 0.30;
        assert!(!should_start_fixed_swing(0.50, swing_duration), "too early");
        assert!(should_start_fixed_swing(0.30, swing_duration), "exactly at duration");
        assert!(should_start_fixed_swing(0.10, swing_duration), "inside window");
        assert!(
            !should_start_fixed_swing(defaults::MIN_TIME_TO_GO_SECS * 0.5, swing_duration),
            "degenerate tti"
        );
        assert!(!should_start_fixed_swing(f64::NAN, swing_duration), "non-finite");
    }

    #[test]
    fn midpoint_strategy_is_exactly_half_duration() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let trajectory = plan_fixed_swing(&robot.arm, rail_x).expect("fixed swing plan");
        let impact_time = fixed_swing_impact_time_secs(
            &robot.arm,
            rail_x,
            &trajectory,
            ImpactTimeStrategy::Midpoint,
        );
        assert!((impact_time - trajectory.duration_secs * 0.5).abs() < 1e-9);
    }

    #[test]
    fn peak_speed_strategy_picks_a_time_strictly_inside_the_swing() {
        let robot = crate::defaults::robot().expect("robot");
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let trajectory = plan_fixed_swing(&robot.arm, rail_x).expect("fixed swing plan");
        let impact_time = fixed_swing_impact_time_secs(
            &robot.arm,
            rail_x,
            &trajectory,
            ImpactTimeStrategy::PeakRacketSpeed,
        );
        assert!(impact_time > 0.0, "impact_time={impact_time}");
        assert!(
            impact_time < trajectory.duration_secs,
            "impact_time={impact_time} duration={}",
            trajectory.duration_secs
        );
    }

    #[test]
    fn should_start_fixed_swing_now_gates_on_impact_time_not_full_duration() {
        // 회귀 방지: `duration_secs`(전체 소요)가 아니라 그보다 짧은
        // `impact_time_secs`(스윙 내부 임팩트 시각)를 기준으로 삼아야, 스윙이
        // "끝나는" 시점이 아니라 "공을 맞히는" 시점에 남은 시간을 맞춘다.
        let duration_secs = 0.53;
        let impact_time_secs = duration_secs * 0.5;
        // 남은 시간이 절반(임팩트 시각)보다 크면 아직 시작하면 안 된다 — 예전
        // 로직(`duration_secs` 기준)이었다면 이 값에서 이미 시작했을 것이다.
        assert!(!should_start_fixed_swing(0.45, impact_time_secs));
        assert!(should_start_fixed_swing(impact_time_secs, impact_time_secs));
        assert!(should_start_fixed_swing(0.10, impact_time_secs));
    }
}
