//! 순수 물리·스윙 계획 (plan §7).

use nalgebra::Vector3;

use crate::error::{DomainError, SwingPlanError};
use crate::impact::{DEFAULT_RESTITUTION, loft_return_velocity, required_racket_velocity};
use crate::robot::Arm;
use crate::types::{Joints, RailMotion, RobotPose, SwingTrajectory, Target};

/// 중력 가속도 [m/s²]
pub const G: Vector3<f64> = Vector3::new(0.0, 0.0, -9.81);

/// 스윙을 시작하기 위해 필요한 최소 리드 타임 [s].
/// 슈터( y≈2.6 )→접수 평면( y≈1.0 ) 비행이 ~0.2s라 0.25s는 너무 큼.
pub const MIN_SWING_SECS: f64 = 0.08;

/// §7.4 실행 가능성 근사 — 관절 각가속도 상한 [rad/s²] (토크 모델 전 스텁).
/// ~0.2s 리드·리니어+팔 동시 이동에서 임팩트 속도를 남기려면 여유가 필요하다.
pub const MAX_JOINT_ACCEL: f64 = 120.0;

/// 공기 저항을 포함한 공 가속도 [m/s²].
pub fn accel(velocity: Vector3<f64>, drag_coefficient: f64) -> Vector3<f64> {
    return G - drag_coefficient * velocity.norm() * velocity;
}

/// 타겟 예측·현재 포즈로 quintic 스윙 궤적을 계획한다 (plan §7.1–§7.5).
pub fn plan_swing(
    arm: &Arm,
    target: Target,
    start: &RobotPose,
) -> Result<SwingTrajectory, DomainError> {
    let prediction = target.prediction;
    let time_to_impact = prediction.time_to_impact_secs;
    if time_to_impact < MIN_SWING_SECS {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::InsufficientTime {
                time_to_impact_secs: time_to_impact,
                min_swing_secs: MIN_SWING_SECS,
            },
        ));
    }

    let (rail_end, impact_position, rail_motion) = if let Some(rail) = &arm.rail {
        let (rail_end, impact) = arm.clamp_impact_for_rail(rail, prediction.impact_position);
        let motion = RailMotion {
            start: start.rail_x,
            end: rail_end,
            start_velocity: 0.0,
            end_velocity: 0.0,
        };
        (rail_end, impact, motion)
    } else {
        let impact = arm.clamp_to_reach(prediction.impact_position);
        (start.rail_x, impact, RailMotion::fixed(start.rail_x))
    };

    let v_in = prediction.incoming_velocity;
    let v_out = loft_return_velocity(impact_position, v_in);

    let end = if let Some(rail) = &arm.rail {
        arm.inverse_kinematics_with_rail(rail, rail_end, impact_position, Some(&start.joints))
    } else {
        arm.inverse_kinematics_near(impact_position, Some(&start.joints))
    }
    .map_err(DomainError::InfeasibleSwing)?;

    let pose = if arm.rail.is_some() {
        arm.forward_kinematics_with_rail(rail_end, &end)
    } else {
        arm.forward_kinematics(&end)
    }
    .ok_or(DomainError::InfeasibleSwing(
        SwingPlanError::InverseKinematicsNoSolution {
            target_x: prediction.impact_position.v.x,
            target_y: prediction.impact_position.v.y,
            target_z: prediction.impact_position.v.z,
        },
    ))?;

    let v_r = required_racket_velocity(v_in, v_out, pose.normal, DEFAULT_RESTITUTION)
        .map_err(DomainError::InfeasibleSwing)?;

    let start_velocity = vec![0.0; start.joints.values.len()];

    let end_velocity = arm
        .joint_velocities_for_ee_velocity(&end, v_r)
        .map_err(DomainError::InfeasibleSwing)?;

    let trajectory = build_feasible_trajectory(
        arm,
        &start.joints,
        end,
        start_velocity,
        end_velocity,
        time_to_impact,
        rail_motion,
    )
    .map_err(DomainError::InfeasibleSwing)?;

    return Ok(trajectory);
}

/// sim 접촉용 — 목표 **위치**에만 도달 (종료 속도·토크 검증 생략).
///
/// `plan_swing`은 리턴 스윙 속도까지 맞추려다 관절 한계에서 부분 궤적(5% 등)로
/// 떨어져 라켓이 공 앞을 지나갈 수 있다. 접촉 검증 단계에서는 위치만 맞춘다.
pub fn plan_contact_swing(
    arm: &Arm,
    target: Target,
    start: &RobotPose,
) -> Result<SwingTrajectory, DomainError> {
    let prediction = target.prediction;
    let time_to_impact = prediction.time_to_impact_secs;
    if time_to_impact < MIN_SWING_SECS {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::InsufficientTime {
                time_to_impact_secs: time_to_impact,
                min_swing_secs: MIN_SWING_SECS,
            },
        ));
    }

    let (rail_end, impact_position, rail_motion) = if let Some(rail) = &arm.rail {
        let (rail_end, impact) = arm.clamp_impact_for_rail(rail, prediction.impact_position);
        let motion = RailMotion {
            start: start.rail_x,
            end: rail_end,
            start_velocity: 0.0,
            end_velocity: 0.0,
        };
        (rail_end, impact, motion)
    } else {
        let impact = arm.clamp_to_reach(prediction.impact_position);
        (start.rail_x, impact, RailMotion::fixed(start.rail_x))
    };

    let end = if let Some(rail) = &arm.rail {
        arm.inverse_kinematics_with_rail(rail, rail_end, impact_position, Some(&start.joints))
    } else {
        arm.inverse_kinematics_near(impact_position, Some(&start.joints))
    }
    .map_err(DomainError::InfeasibleSwing)?;

    let zero = vec![0.0; start.joints.values.len()];
    return Ok(SwingTrajectory::new(
        start.joints.clone(),
        end,
        zero.clone(),
        zero,
        time_to_impact,
        rail_motion,
    ));
}

/// 속도·가속 한계 안에 들어오는 quintic을 만든다.
///
/// 종료 **위치**는 항상 임팩트 IK 해. 끝속도는 한계 안으로 스케일하되,
/// 타격 모드에서는 **0으로 버리지 않는다** (최소 스케일 유지).
fn build_feasible_trajectory(
    arm: &Arm,
    start: &Joints,
    end: Joints,
    start_velocity: Vec<f64>,
    end_velocity: Vec<f64>,
    duration: f64,
    rail: RailMotion,
) -> Result<SwingTrajectory, SwingPlanError> {
    let fitted = fit_end_velocity(
        arm,
        start,
        &end,
        &start_velocity,
        end_velocity,
        duration,
        rail,
    );
    return Ok(SwingTrajectory::new(
        start.clone(),
        end,
        start_velocity,
        fitted,
        duration,
        rail,
    ));
}

fn trajectory_within_limits(arm: &Arm, trajectory: &SwingTrajectory) -> bool {
    let joints_ok = trajectory.peak_joint_speed() <= arm.max_joint_speed
        && trajectory.peak_joint_acceleration() <= MAX_JOINT_ACCEL;
    let rail_ok = arm
        .rail
        .as_ref()
        .map_or(true, |rail| trajectory.peak_rail_speed() <= rail.max_speed);
    return joints_ok && rail_ok;
}

/// quintic이 관절 한계 안에 들어오도록 임팩트 각속도를 점진적으로 줄인다 (§7.4 근사).
fn fit_end_velocity(
    arm: &Arm,
    start: &Joints,
    end: &Joints,
    start_velocity: &[f64],
    mut end_velocity: Vec<f64>,
    duration: f64,
    rail: RailMotion,
) -> Vec<f64> {
    for _ in 0..32 {
        let trajectory = SwingTrajectory::new(
            start.clone(),
            end.clone(),
            start_velocity.to_vec(),
            end_velocity.clone(),
            duration,
            rail,
        );
        if trajectory_within_limits(arm, &trajectory) {
            return end_velocity;
        }

        let peak_speed = trajectory.peak_joint_speed();
        let peak_accel = trajectory.peak_joint_acceleration();
        let speed_scale = if peak_speed > arm.max_joint_speed {
            arm.max_joint_speed / peak_speed * 0.95
        } else {
            1.0
        };
        let accel_scale = if peak_accel > MAX_JOINT_ACCEL {
            MAX_JOINT_ACCEL / peak_accel * 0.95
        } else {
            1.0
        };
        let scale = speed_scale.min(accel_scale);
        if scale >= 0.99 {
            break;
        }
        for v in &mut end_velocity {
            *v *= scale;
        }
    }

    // 한계를 완전히 못 맞춰도 끝속도를 0으로 버리지 않는다 (타격 의도 유지).
    return end_velocity;
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::constants::table;
    use crate::robot::Arm;
    use crate::types::Prediction;

    fn sample_three_dof_arm() -> Arm {
        return Arm::competition().expect("테스트용 3DOF arm");
    }

    fn sample_start(arm: &Arm) -> RobotPose {
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        return RobotPose::new(rail_x, arm.default_joints.clone());
    }

    fn sample_target(time_to_impact_secs: f64) -> Target {
        let arm = sample_three_dof_arm();
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let impact_position = arm
            .forward_kinematics_with_rail(rail_x, &arm.default_joints)
            .expect("기본 자세 FK")
            .position;
        return Target {
            prediction: Prediction {
                time_to_impact_secs,
                impact_position,
                incoming_velocity: Vector3::new(0.0, -4.0, -0.2),
            },
        };
    }

    #[test]
    fn plan_swing_reaches_impact_with_end_velocity() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let target = sample_target(0.35);
        let trajectory = plan_swing(&arm, target, &start).expect("스윙 계획");
        let pose = arm
            .forward_kinematics_with_rail(trajectory.rail.end, trajectory.goal_joints())
            .expect("FK");
        assert!((pose.position.v - target.prediction.impact_position.v).norm() < 1e-5);
        assert!(
            trajectory.end_velocity.iter().any(|v| v.abs() > 0.05),
            "로프트 타격 끝속도가 살아 있어야 함: {:?}",
            trajectory.end_velocity
        );
        assert!(trajectory.peak_joint_speed() <= arm.max_joint_speed * 1.05);
    }

    #[test]
    fn plan_contact_swing_is_position_only() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let target = sample_target(0.35);
        let trajectory = plan_contact_swing(&arm, target, &start).expect("contact");
        assert!(trajectory.end_velocity.iter().all(|v| *v == 0.0));
        let pose = arm
            .forward_kinematics_with_rail(trajectory.rail.end, trajectory.goal_joints())
            .expect("FK");
        assert!((pose.position.v - target.prediction.impact_position.v).norm() < 1e-5);
    }

    #[test]
    fn plan_swing_moves_rail_to_impact_x() {
        let arm = sample_three_dof_arm();
        let start = RobotPose::new(0.1, arm.default_joints.clone());
        let reachable = arm
            .forward_kinematics_with_rail(table::WIDTH_X * 0.8, &arm.default_joints)
            .expect("FK")
            .position;
        let impact =
            crate::types::Point3::new(reachable.v.x, table::DEFAULT_HIT_PLANE_Y, reachable.v.z);
        let target = Target {
            prediction: Prediction {
                time_to_impact_secs: 0.3,
                impact_position: impact,
                incoming_velocity: Vector3::new(0.0, -5.0, -0.2),
            },
        };
        let trajectory = plan_swing(&arm, target, &start).expect("스윙 계획");
        assert!((trajectory.rail.end - impact.v.x).abs() < 1e-6);
        assert!((trajectory.rail.start - 0.1).abs() < 1e-6);
    }

    #[test]
    fn plan_swing_fails_when_insufficient_time() {
        let arm = sample_three_dof_arm();
        let err = plan_swing(&arm, sample_target(0.05), &sample_start(&arm)).unwrap_err();
        let DomainError::InfeasibleSwing(SwingPlanError::InsufficientTime {
            time_to_impact_secs,
            min_swing_secs,
        }) = err
        else {
            panic!("InsufficientTime 기대");
        };
        assert!((time_to_impact_secs - 0.05).abs() < f64::EPSILON);
        assert!((min_swing_secs - MIN_SWING_SECS).abs() < f64::EPSILON);
    }

    #[test]
    fn competition_geometry_reachable_with_rail() {
        let arm = Arm::competition().expect("competition arm");

        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let far_impact = arm
            .forward_kinematics_with_rail(rail_x, &arm.default_joints)
            .expect("FK")
            .position;
        let start = RobotPose::new(rail_x, arm.default_joints.clone());
        let target = Target {
            prediction: Prediction {
                time_to_impact_secs: 0.22,
                impact_position: far_impact,
                incoming_velocity: Vector3::new(0.0, -7.5, -0.3),
            },
        };
        let trajectory = plan_swing(&arm, target, &start).expect("슈터→로봇 기본 샷");
        assert!((trajectory.rail.end - far_impact.v.x).abs() < 1e-6);
        assert!(trajectory.peak_joint_speed() <= arm.max_joint_speed);
        assert_ne!(
            trajectory.goal_joints().values,
            arm.default_joints.values,
            "접수 방향으로 관절 목표가 달라져야 함"
        );
    }
}
