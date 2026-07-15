//! 순수 물리/스윙 계획.

use nalgebra::Vector3;

use super::impact::{rally_return_velocity, required_racket_velocity};
use crate::constants::{
    DEFAULT_RESTITUTION, G, JOINT_INERTIA, MAX_JOINT_ACCEL, MAX_JOINT_TORQUE, MIN_SWING_SECS,
    SWING_COMMIT_MAX_BALL_Y_FRAC, SWING_COMMIT_MAX_SECS, table,
};
use crate::error::{DomainError, SwingPlanError};
use crate::robot::Arm;
use crate::types::{Joints, Prediction, RailMotion, RobotPose, SwingTrajectory};

/// 공기 저항을 포함한 공 가속도 [m/s^2].
pub fn accel(velocity: Vector3<f64>, drag_coefficient: f64) -> Vector3<f64> {
    return G - drag_coefficient * velocity.norm() * velocity;
}

/// 임팩트까지 남은 시간이 스윙 commit 창 `[MIN_SWING, COMMIT_MAX]` 안인지.
///
/// 창보다 이르면 대기(발사 직후 긴 궤적 금지), 짧으면 `InsufficientTime`.
pub fn in_swing_commit_window(time_to_impact_secs: f64) -> bool {
    return (MIN_SWING_SECS..=SWING_COMMIT_MAX_SECS).contains(&time_to_impact_secs);
}

/// 네트 통과 후인지 - ground truth/EKF control 공통 commit 게이트.
pub fn ball_past_midcourt_for_commit(ball_y: f64) -> bool {
    return ball_y <= table::LENGTH_Y * SWING_COMMIT_MAX_BALL_Y_FRAC;
}

/// 예측/현재 포즈로 quintic 스윙 궤적을 계획한다.
pub fn plan_swing(
    arm: &Arm,
    prediction: Prediction,
    start: &RobotPose,
) -> Result<SwingTrajectory, DomainError> {
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
    let v_out = rally_return_velocity(impact_position, v_in);

    // 라켓 open을 IK seed에 먼저 반영한다. URDF 손목은 EE offset도 움직이므로
    // 위치 IK 뒤에 관절값을 덮어쓰면 실제 타격점이 어긋난다.
    let ik_hint = arm
        .with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out))
        .map_err(DomainError::InfeasibleSwing)?;
    let mut end = if let Some(rail) = &arm.rail {
        arm.inverse_kinematics_with_rail(rail, rail_end, impact_position, Some(&ik_hint))
    } else {
        arm.inverse_kinematics_near(impact_position, Some(&ik_hint))
    }
    .map_err(DomainError::InfeasibleSwing)?;

    // 임팩트 자세가 테이블을 뚫지 않게 OBB 클램프
    end = crate::planner::collision::clamp_above_table(arm, rail_end, &end);

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

    return build_feasible_trajectory(
        arm,
        &start.joints,
        end,
        start_velocity,
        end_velocity,
        time_to_impact,
        rail_motion,
    )
    .map_err(DomainError::InfeasibleSwing);
}

/// 속도/가속 한계 안에 들어오는 quintic을 만든다.
///
/// 종료 위치는 항상 임팩트 IK 해. 끝속도는 한계 안으로 스케일하되
/// 타격 모드에서는 0으로 버리지 않는다 (최소 스케일 유지).
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

fn peak_required_torque(trajectory: &SwingTrajectory) -> f64 {
    return JOINT_INERTIA * trajectory.peak_joint_acceleration();
}

fn trajectory_within_limits(arm: &Arm, trajectory: &SwingTrajectory) -> bool {
    let joints_ok = trajectory.peak_joint_speed() <= arm.max_joint_speed
        && trajectory.peak_joint_acceleration() <= MAX_JOINT_ACCEL
        && peak_required_torque(trajectory) <= MAX_JOINT_TORQUE;
    let rail_ok = arm
        .rail
        .as_ref()
        .map_or(true, |rail| trajectory.peak_rail_speed() <= rail.max_speed);
    return joints_ok && rail_ok;
}

/// quintic이 관절 한계 안에 들어오도록 임팩트 각속도를 점진적으로 줄인다 ( 근사).
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
        let peak_torque = peak_required_torque(&trajectory);
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
        let torque_scale = if peak_torque > MAX_JOINT_TORQUE {
            MAX_JOINT_TORQUE / peak_torque * 0.95
        } else {
            1.0
        };
        let scale = speed_scale.min(accel_scale).min(torque_scale);
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
        return Arm::competition().expect("테스트용 4DOF arm");
    }

    fn sample_start(arm: &Arm) -> RobotPose {
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        return RobotPose::new(rail_x, arm.default_joints.clone());
    }

    fn sample_prediction(time_to_impact_secs: f64) -> Prediction {
        let arm = sample_three_dof_arm();
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let impact_position = arm
            .forward_kinematics_with_rail(rail_x, &arm.default_joints)
            .expect("기본 자세 FK")
            .position;
        return Prediction {
            time_to_impact_secs,
            impact_position,
            incoming_velocity: Vector3::new(0.0, -4.0, -0.2),
        };
    }

    #[test]
    fn in_swing_commit_window_bounds() {
        assert!(!in_swing_commit_window(0.05));
        assert!(in_swing_commit_window(0.12));
        assert!(in_swing_commit_window(SWING_COMMIT_MAX_SECS));
        assert!(!in_swing_commit_window(0.35));
    }

    #[test]
    fn midcourt_gate_matches_fraction() {
        use crate::constants::control::SWING_COMMIT_MAX_BALL_Y_FRAC;
        let limit = table::LENGTH_Y * SWING_COMMIT_MAX_BALL_Y_FRAC;
        assert!(!ball_past_midcourt_for_commit(limit + 0.01));
        assert!(ball_past_midcourt_for_commit(limit));
        assert!(ball_past_midcourt_for_commit(0.3));
    }

    #[test]
    fn plan_swing_reaches_impact_with_end_velocity() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let prediction = sample_prediction(0.35);
        let trajectory = plan_swing(&arm, prediction, &start).expect("스윙 계획");
        let pose = arm
            .forward_kinematics_with_rail(trajectory.rail.end, trajectory.goal_joints())
            .expect("FK");
        assert!((pose.position.v.x - prediction.impact_position.v.x).abs() < 1e-4);
        assert!((pose.position.v.y - prediction.impact_position.v.y).abs() < 1e-4);
        assert!(
            pose.position.v.z + 1e-4 >= prediction.impact_position.v.z,
            "테이블 클램프로 z만 올라갈 수 있음"
        );
        assert!(
            crate::planner::collision::table_penetration(
                &arm,
                trajectory.rail.end,
                trajectory.goal_joints()
            ) < 1e-3
        );
        assert!(
            trajectory.end_velocity.iter().any(|v| v.abs() > 0.05),
            "로프트 타격 끝속도가 살아 있어야 함: {:?}",
            trajectory.end_velocity
        );
        assert!(trajectory.peak_joint_speed() <= arm.max_joint_speed * 1.05);
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
        let prediction = Prediction {
            time_to_impact_secs: 0.3,
            impact_position: impact,
            incoming_velocity: Vector3::new(0.0, -5.0, -0.2),
        };
        let trajectory = plan_swing(&arm, prediction, &start).expect("스윙 계획");
        assert!((trajectory.rail.end - impact.v.x).abs() < 1e-6);
        assert!((trajectory.rail.start - 0.1).abs() < 1e-6);
    }

    #[test]
    fn plan_swing_fails_when_insufficient_time() {
        let arm = sample_three_dof_arm();
        let err = plan_swing(&arm, sample_prediction(0.05), &sample_start(&arm)).unwrap_err();
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
        let prediction = Prediction {
            time_to_impact_secs: 0.22,
            impact_position: far_impact,
            incoming_velocity: Vector3::new(0.0, -7.5, -0.3),
        };
        let trajectory = plan_swing(&arm, prediction, &start).expect("슈터->로봇 기본 샷");
        assert!((trajectory.rail.end - far_impact.v.x).abs() < 1e-6);
        assert!(trajectory.peak_joint_speed() <= arm.max_joint_speed);
        assert_ne!(
            trajectory.goal_joints().values,
            arm.default_joints.values,
            "접수 방향으로 관절 목표가 달라져야 함"
        );
    }
}
