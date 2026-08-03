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

/// 남은 시간이 고정 스윙 소요 시간 이하가 되는 즉시 스윙을 시작해야 한다.
/// 소요 시간이 고정이라(끝속도 탐색 없음) `plan_swing`처럼 duration을
/// 남은 시간에 맞춰 늘리지 않는다 — 대신 "지금이 그 타이밍인가"만 본다.
pub fn should_start_fixed_swing(time_to_impact_secs: f64, swing_duration_secs: f64) -> bool {
    return time_to_impact_secs.is_finite()
        && time_to_impact_secs > defaults::MIN_TIME_TO_GO_SECS
        && time_to_impact_secs <= swing_duration_secs;
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
}
