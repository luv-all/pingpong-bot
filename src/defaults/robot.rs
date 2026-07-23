//! 활성 로봇 · URDF 프리셋.
//!
//! 런타임이 쓰는 것은 [`robot`]. 바꾸려면 그 본문만 고친다.
//! 리니어모터 철제 프레임 위치는 [`rail_frame`].
//!
//! 공유·배선은 항상 [`Robot`] (`shared_robot`). FK/IK가 필요하면 `robot.arm`을 본다.

use std::path::PathBuf;
use std::sync::Arc;

use nalgebra::{Isometry3, UnitQuaternion, Vector3};

use crate::constants::geometry;
use crate::constants::table;
use crate::robot::{
    Arm, JointLimit, Joints, MountPreset, RailFrame, Robot, RobotBuildError, RobotBuilder,
    SerialChain, SerialJoint,
};

/// 리니어모터를 받치는 철제 프로파일 (탁구대 끝면·윗면 기준).
///
/// 실측: 끝면 뒤 20 cm, 윗면 위 20 cm.
pub fn rail_frame() -> RailFrame {
    return RailFrame {
        behind_table_end: 0.20,
        above_table: 0.20,
    };
}

/// 경연용 단순 4-dof (URDF 없음) → [`Robot`].
///
/// mesh가 필요하면 [`urdf_4dof`]. 활성 배선은 [`robot`].
pub fn primitive_4dof() -> Result<Robot, RobotBuildError> {
    const MAX_JOINT_SPEED: f64 = 16.0;
    const RAIL_MAX_SPEED: f64 = 12.0;

    let frame = rail_frame();
    let mount_y = frame.mount_y();
    let mount_z = frame.mount_z();

    let joints = vec![
        SerialJoint::new(
            Isometry3::translation(-0.02575, 0.028, 0.0601),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q0 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0255, 0.0, 0.0825),
            Vector3::new(0.0, 0.0, -1.0),
        )
        .expect("4-dof q1 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0, 0.025, 0.1398),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q2 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0, 0.1518, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q3 axis"),
    ];

    let chain = SerialChain::new(
        UnitQuaternion::identity(),
        joints,
        // CAD tip: +Y=면 법선, −Z=손잡이(면 내, 홈 포즈 기준). 타격점은 면 평행 이동.
        Isometry3::translation(
            0.0,
            -geometry::RACKET_HALF_Z,
            -geometry::RACKET_HANDLE_LENGTH,
        ),
    )
    .expect("4-dof serial chain");

    let built = Arm::builder()
        .base_xyz(0.0, mount_y, mount_z)
        .linear_rail(mount_y, mount_z, 0.0, table::WIDTH_X, RAIL_MAX_SPEED)
        .serial_chain(
            chain,
            vec![
                None,
                Some(JointLimit::new(-0.523599, 0.523599)),
                Some(JointLimit::new(-2.007129, 1.48353)),
                Some(JointLimit::new(-2.094395, 2.094395)),
            ],
            Joints::from_slice(&[0.0, 0.0, -0.2617995, 0.0]),
        )
        .max_joint_speed(MAX_JOINT_SPEED)
        .build()
        .map_err(|e| RobotBuildError::ArmConversion {
            reason: e.to_string(),
        })?;

    return Ok(Robot::from_arm(built));
}

/// `robot()`을 `Arc`로 (파이프라인·테스트용).
pub fn shared_robot() -> Arc<Robot> {
    return Arc::new(robot().expect("defaults::robot"));
}

/// 호환 별칭 — `primitive_4dof().arm` (호출부 이전용, 곧 제거).
pub fn arm() -> Result<Arm, RobotBuildError> {
    return Ok((*primitive_4dof()?.arm).clone());
}

/// 호환 별칭 — `shared_robot().arm`.
pub fn shared_arm() -> Arc<Arm> {
    return Arc::clone(&shared_robot().arm);
}

/// `assets/robots/4-dof` URDF 프리셋 (진단·비교용).
pub fn urdf_4dof() -> Result<Robot, RobotBuildError> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("assets/robots/4-dof/urdf/all-4-export.urdf");
    return RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(Some("pingpong_paddle_v5_1"))
        .mount_preset(MountPreset::Rep103AtTableEnd)
        .max_joint_speed(16.0)
        .build();
}

/// `assets/robots/urdf-test` 프리셋.
pub fn urdf_test() -> Result<Robot, RobotBuildError> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
    return RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(Some("pingpong_paddle_v5_1"))
        .mount_preset(MountPreset::Rep103AtTableEnd)
        .max_joint_speed(16.0)
        .build();
}

/// **지금 쓰는 로봇.** 바꾸려면 이 함수 본문만 고친다 (`urdf_4dof` 등).
pub fn robot() -> Result<Robot, RobotBuildError> {
    return urdf_4dof();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::table;

    #[test]
    fn rail_frame_mounts_behind_and_above_table() {
        let frame = rail_frame();
        assert!((frame.mount_y() - (-0.20)).abs() < 1e-12);
        assert!((frame.mount_z() - (table::SURFACE_Z + 0.20)).abs() < 1e-12);
        assert_eq!(frame.mount_xyz0(), [0.0, -0.20, table::SURFACE_Z + 0.20]);
    }

    #[test]
    fn primitive_follows_rail_frame() {
        let robot = primitive_4dof().expect("primitive_4dof");
        let arm = robot.arm.as_ref();
        let frame = rail_frame();
        assert!((arm.base.coords.y - frame.mount_y()).abs() < 1e-12);
        assert!((arm.base.coords.z - frame.mount_z()).abs() < 1e-12);
        let rail = arm.rail.expect("rail");
        assert!((rail.mount_y - frame.mount_y()).abs() < 1e-12);
        assert!((rail.mount_z - frame.mount_z()).abs() < 1e-12);
    }
}
