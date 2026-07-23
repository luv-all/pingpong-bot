use nalgebra::Vector3;

use super::*;
use crate::constants::table;
use crate::error::SwingPlanError;
use crate::{Joints, Point3};

fn sample_competition_arm() -> Arm {
    return crate::entry::competition_arm().expect("테스트용 4DOF arm");
}

#[test]
fn wrist_open_tilts_racket_face() {
    let arm = sample_competition_arm();
    let flat = arm
        .with_wrist_open(&arm.default_joints, 0.1)
        .expect("wrist");
    let lofted = arm
        .with_wrist_open(&arm.default_joints, 0.9)
        .expect("wrist");
    let n_flat = arm.forward_kinematics(&flat).expect("FK").normal;
    let n_loft = arm.forward_kinematics(&lofted).expect("FK").normal;
    assert!(
        n_loft.z > n_flat.z,
        "open up -> normal.z up: flat={n_flat:?} loft={n_loft:?}"
    );
}

#[test]
fn racket_face_points_toward_opponent_not_ceiling() {
    let arm = sample_competition_arm();
    let pose = arm.forward_kinematics(&arm.default_joints).expect("FK");
    assert!(
        pose.normal.y > 0.5,
        "면이 상대(+Y)를 봐야 함: normal={:?}",
        pose.normal
    );
    assert!(
        pose.normal.y > pose.normal.z,
        "면이 천장보다 상대 쪽: normal={:?}",
        pose.normal
    );
    // local +Z (얇은 축) ~= normal
    let [w, x, y, z] = pose.orientation;
    let q = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z));
    let local_z = q * Vector3::new(0.0, 0.0, 1.0);
    assert!((local_z - pose.normal).norm() < 1e-5, "local_z={local_z:?}");
}

#[test]
fn builder_produces_three_dof_arm() {
    let arm = sample_competition_arm();
    assert_eq!(arm.joint_count(), 4);
}

#[test]
fn builder_rejects_missing_serial_chain() {
    let err = ArmBuilder::new()
        .base_xyz(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
        .build()
        .unwrap_err();
    assert!(matches!(err, ArmBuildError::MissingSerialChain));
}

#[test]
fn default_arm_produces_racket_pose() {
    let arm = sample_competition_arm();
    let state = arm.initial_state();
    let pose = state.racket_pose(&arm).expect("FK");
    assert!(pose.position.v.y > arm.base.v.y);
    assert!(pose.position.v.z >= arm.base.v.z);
}

#[test]
fn step_moves_angles_toward_targets() {
    let arm = sample_competition_arm();
    let mut state = arm.initial_state();
    state.set_targets(Joints::from_slice(&[0.5, 0.8, -0.2, 0.45]));
    state.step_toward_targets(&arm, 0.1);
    assert_ne!(state.joints().values[0], 0.0);
}

#[test]
fn rejects_wrong_joint_count_in_fk() {
    let arm = sample_competition_arm();
    assert!(
        arm.forward_kinematics(&Joints::from_slice(&[0.0]))
            .is_none()
    );
}

#[test]
fn inverse_kinematics_round_trips_forward_kinematics() {
    let arm = sample_competition_arm();
    let joints = Joints::from_slice(&[0.2, 0.2, -0.3, -0.45]);
    let pose = arm.forward_kinematics(&joints).expect("FK");
    let solved = arm.inverse_kinematics(pose.position).expect("IK");
    let again = arm.forward_kinematics(&solved).expect("FK again");
    assert!((again.position.v - pose.position.v).norm() < 1e-5);
}

#[test]
fn pose_ik_round_trips_position_and_face_normal_with_rail() {
    let arm = crate::entry::competition_arm().expect("arm");
    let expected = RobotPose::new(0.62, Joints::from_slice(&[0.08, 0.12, -0.55, -0.28]));
    let target = arm
        .forward_kinematics_with_rail(expected.rail_x, &expected.joints)
        .expect("target FK");
    let hint = RobotPose::new(0.35, arm.default_joints.clone());

    let solved = arm
        .inverse_pose_with_rail(target.position, target.normal, &hint)
        .expect("pose IK");
    let actual = arm
        .forward_kinematics_with_rail(solved.rail_x, &solved.joints)
        .expect("solved FK");
    assert!((actual.position.v - target.position.v).norm() < 2e-4);
    assert!((actual.normal - target.normal).norm() < 1e-3);
}

#[test]
fn generalized_velocity_ik_moves_center_without_rotating_face() {
    let arm = crate::entry::competition_arm().expect("arm");
    let pose = RobotPose::new(0.62, Joints::from_slice(&[0.08, 0.12, -0.55, -0.28]));
    let before = arm
        .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
        .expect("before FK");
    let desired = Vector3::new(0.08, -0.25, 0.35);
    let (rail_velocity, joint_velocities) = arm
        .velocities_for_racket_velocity(&pose, desired)
        .expect("velocity IK");
    let dt = 1e-5;
    let after_pose = RobotPose::new(
        pose.rail_x + rail_velocity * dt,
        Joints {
            values: pose
                .joints
                .values
                .iter()
                .zip(joint_velocities)
                .map(|(angle, velocity)| angle + velocity * dt)
                .collect(),
        },
    );
    let after = arm
        .forward_kinematics_with_rail(after_pose.rail_x, &after_pose.joints)
        .expect("after FK");
    let actual_velocity = (after.position.v - before.position.v) / dt;
    let normal_velocity = (after.normal - before.normal) / dt;
    assert!((actual_velocity - desired).norm() < 1e-3);
    assert!(normal_velocity.norm() < 1e-3);
}

#[test]
fn generalized_velocity_ik_can_move_inward_from_rail_max() {
    let arm = crate::entry::competition_arm().expect("arm");
    let rail = arm.rail.expect("rail");
    let pose = RobotPose::new(rail.x_max, arm.default_joints.clone());
    let (rail_velocity, _) = arm
        .velocities_for_racket_velocity(&pose, Vector3::new(-0.1, 0.0, 0.0))
        .expect("boundary velocity IK");
    assert!(
        rail_velocity < -0.05,
        "레일 상한에서도 안쪽 속도를 계산해야 함: {rail_velocity}"
    );
}

#[test]
fn inverse_kinematics_rejects_unreachable_target() {
    let arm = sample_competition_arm();
    let err = arm
        .inverse_kinematics(Point3::new(10.0, 10.0, 10.0))
        .unwrap_err();
    assert!(matches!(
        err,
        SwingPlanError::InverseKinematicsNoSolution { .. }
    ));
}

#[test]
fn clamp_impact_preserves_hit_plane_y_when_possible() {
    let arm = sample_competition_arm();
    let rail = arm.rail.as_ref().expect("competition rail");
    let far = Point3::new(0.76, 0.30, table::SURFACE_Z + 0.25);
    let (rail_x, clamped) = arm.clamp_impact_for_rail(rail, far);
    assert!(
        (clamped.v.y - 0.30).abs() < 1e-9,
        "도달 밖이어도 y는 hit plane 유지: {}",
        clamped.v.y
    );
    let mount = rail.mount_point(rail_x);
    let max_reach = arm.arm_length();
    assert!((clamped.v - mount.v).norm() <= max_reach + 1e-6);
}
