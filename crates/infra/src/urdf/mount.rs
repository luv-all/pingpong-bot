//! URDF(REP-103) 좌표계 → sim(Z-up) 배치.
//!
//! # 좌표계
//!
//! | | URDF / ROS REP-103 | pingpong-bot sim |
//! |---|---|---|
//! | 위 | **+Z** | **+Z** |
//! | 앞 | +X | +Y (테이블 길이) |
//! | 좌 | +Y | −X (테이블 너비) |
//!
//! 원점 = 탁구대 로봇 쪽 꼭짓점. **X** = 너비, **Y** = 길이, **Z** = 고도.
//!
//! # 변환
//!
//! sim도 REP-103과 같이 **Z-up**이면 URDF `base_link` 축이 sim 축과 일치한다 (`rpy = 0`).
//! (이전 Y-up sim에서는 `Rx(−90°)·Ry(−90°)` 보정이 필요했음.)

use nalgebra::Isometry3;

use pingpong_domain::constants::table;

use super::fk;

/// sim에서 URDF 루트를 올릴 위치·자세.
#[derive(Debug, Clone, Copy)]
pub struct SimRobotMount {
    /// sim 월드 위치 [m] — `base_link` 원점
    pub position: [f64; 3],
    /// URDF → sim 회전 RPY [rad]
    pub rpy: [f64; 3],
}

impl SimRobotMount {
    /// 이미 sim Z-up으로 작성된 URDF (내장 `competition_arm` 등).
    pub fn competition_placed() -> Self {
        return Self {
            position: [table::WIDTH_X * 0.15, 0.02, table::SURFACE_Z],
            rpy: [0.0, 0.0, 0.0],
        };
    }

    /// REP-103 Z-up URDF → sim Z-up. `base_link` z=0이 탁구대 윗면에 닿도록 배치.
    pub fn rep103_z_up_at_table_end() -> Self {
        return Self {
            position: [table::WIDTH_X * 0.15, 0.02, table::SURFACE_Z],
            rpy: [0.0, 0.0, 0.0],
        };
    }

    pub(crate) fn isometry(self) -> Isometry3<f64> {
        return fk::mount_to_iso(self.position, self.rpy);
    }
}

pub(crate) fn default_sim_mount(robot_name: &str) -> SimRobotMount {
    if robot_name == "urdf-test" {
        return SimRobotMount::rep103_z_up_at_table_end();
    }
    return SimRobotMount::competition_placed();
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::urdf::UrdfRobot;

    #[test]
    fn rep103_mount_puts_base_on_table_and_arm_toward_plus_y() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
        if !path.exists() {
            return;
        }
        let urdf =
            UrdfRobot::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load urdf-test");
        assert_eq!(urdf.mount.rpy, [0.0, 0.0, 0.0]);

        let joints = urdf.default_joints().values;
        let base = urdf
            .link_poses_in_sim(joints.as_slice())
            .into_iter()
            .find(|(n, _, _)| n == "base_link")
            .map(|(_, p, _)| p)
            .expect("base");
        let ee = urdf
            .end_effector_pose_in_sim(joints.as_slice())
            .expect("ee");

        assert!(
            (base[2] - table::SURFACE_Z).abs() < 0.01,
            "base z≈탁구대 면: {}",
            base[2]
        );
        assert!(base[1] < 0.15, "base y≈로봇 끝: {}", base[1]);
        assert!(
            ee.position.v.y > base[1],
            "팔 +y(테이블): base_y={} ee_y={}",
            base[1],
            ee.position.v.y
        );
        assert!(
            ee.position.v.z >= base[2] - 0.02,
            "베이스가 탁구대 아래로 꺼지지 않음: base_z={} ee_z={}",
            base[2],
            ee.position.v.z
        );
    }
}
