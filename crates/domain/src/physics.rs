//! 순수 물리·스윙 계획 (1단계 스텁).

use nalgebra::Vector3;

use crate::error::{DomainError, SwingPlanError};
use crate::robot::Arm;
use crate::types::{Joints, SwingTrajectory, Target};

/// 중력 가속도 [m/s²]
pub const G: Vector3<f64> = Vector3::new(0.0, 0.0, -9.81);

/// 스윙을 시작하기 위해 필요한 최소 시간 [s].
pub const MIN_SWING_SECS: f64 = 0.25;

/// 공기 저항을 포함한 공 가속도 [m/s²].
pub fn accel(velocity: Vector3<f64>, drag_coefficient: f64) -> Vector3<f64> {
    return G - drag_coefficient * velocity.norm() * velocity;
}

/// 타겟 예측을 바탕으로 스윙 궤적을 계획한다.
pub fn plan_swing(arm: &Arm, target: Target) -> Result<SwingTrajectory, DomainError> {
    let time_to_impact = target.prediction.time_to_impact_secs;
    if time_to_impact < MIN_SWING_SECS {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::InsufficientTime {
                time_to_impact_secs: time_to_impact,
                min_swing_secs: MIN_SWING_SECS,
            },
        ));
    }

    let joints = Joints::from_slice(&[0.0, 0.5, -0.3]);
    if !arm.joints_in_limits(&joints) {
        let (min, max) = arm
            .limits
            .first()
            .map(|l| (l.min, l.max))
            .unwrap_or((0.0, 0.0));
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::JointLimit {
                joint_index: 0,
                value: joints.values[0],
                min,
                max,
            },
        ));
    }

    return Ok(SwingTrajectory {
        joints,
        duration_secs: MIN_SWING_SECS,
    });
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::constants::table;
    use crate::robot::Arm;
    use crate::types::{Point3, Prediction};

    fn sample_target(time_to_impact_secs: f64) -> Target {
        return Target {
            prediction: Prediction {
                time_to_impact_secs,
                impact_position: Point3::new(0.1, 1.0, table::SURFACE_Z),
                incoming_velocity: Vector3::new(0.0, -1.0, 0.0),
            },
        };
    }

    fn sample_three_dof_arm() -> Arm {
        return Arm::builder()
            .base_xyz(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
            .link(0.35)
            .revolute_at(-1.2, 1.2, 0.0)
            .link(0.30)
            .revolute_at(-0.2, 1.4, 0.6)
            .link(0.15)
            .revolute_at(-1.5, 0.5, -0.4)
            .max_joint_speed(2.5)
            .build()
            .expect("테스트용 3DOF arm");
    }

    #[test]
    fn plan_swing_ok_when_enough_time() {
        let arm = sample_three_dof_arm();
        plan_swing(&arm, sample_target(0.3)).expect("충분한 시간");
    }

    #[test]
    fn plan_swing_fails_when_insufficient_time() {
        let arm = sample_three_dof_arm();
        let err = plan_swing(&arm, sample_target(0.1)).unwrap_err();
        let DomainError::InfeasibleSwing(SwingPlanError::InsufficientTime {
            time_to_impact_secs,
            min_swing_secs,
        }) = err
        else {
            panic!("InsufficientTime 기대");
        };
        assert!((time_to_impact_secs - 0.1).abs() < f64::EPSILON);
        assert!((min_swing_secs - MIN_SWING_SECS).abs() < f64::EPSILON);
    }
}
