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

use super::fk;
use crate::constants::table;

/// sim에서 URDF 루트를 올릴 위치·자세.
#[derive(Debug, Clone, Copy)]
pub struct SimRobotMount {
    /// sim 월드 위치 [m] — `base_link` 원점
    pub position: [f64; 3],
    /// URDF → sim 회전 RPY [rad]
    pub rpy: [f64; 3],
}

impl SimRobotMount {
    /// [`crate::robot::RailFrame`] 위 — `base_link` 원점 = 프로파일 마운트.
    pub fn on_rail_frame(frame: crate::robot::RailFrame) -> Self {
        return Self {
            position: frame.mount_xyz0(),
            rpy: [0.0, 0.0, 0.0],
        };
    }

    /// 이미 sim Z-up으로 작성된 URDF — [`crate::defaults::rail_frame`] 배치.
    pub fn competition_placed() -> Self {
        return Self::on_rail_frame(crate::defaults::rail_frame());
    }

    /// REP-103 Z-up URDF → sim Z-up — [`crate::defaults::rail_frame`] 배치.
    pub fn rep103_z_up_at_table_end() -> Self {
        return Self::on_rail_frame(crate::defaults::rail_frame());
    }

    /// [`Self::rep103_z_up_at_table_end`]와 같지만 베이스 위치(테이블과의
    /// 거리·높이)를 직접 지정한다 — `tools/mount_search`류 마운트 스윕 전용.
    ///
    /// `base_y`: 베이스 y [m], 탁구대 로봇쪽 끝(y=0) 기준.
    /// `height_offset_m`: 탁구대 면(`table::SURFACE_Z`) 대비 높이 오프셋 [m].
    ///
    /// primitive 팔의 [`crate::defaults::primitive_4dof_with_mount`]와 같은
    /// 좌표 관례를 쓴다. 다만 두 모델은 링크 길이가 달라 최적 마운트 위치가
    /// 그대로 옮겨지지 않으므로, URDF 로봇은 URDF 로봇대로 스윕해야 한다.
    pub fn rep103_z_up_at_table_end_with_mount(base_y: f64, height_offset_m: f64) -> Self {
        return Self {
            position: [0.0, base_y, table::SURFACE_Z + height_offset_m],
            rpy: [0.0, 0.0, 0.0],
        };
    }

    pub(crate) fn isometry(self) -> Isometry3<f64> {
        return fk::mount_to_iso(self.position, self.rpy);
    }
}

pub(crate) fn default_sim_mount(robot_name: &str) -> SimRobotMount {
    // CAD/Onshape export (REP-103 Z-up) — 철제 프로파일(`defaults::rail_frame`) 위 배치
    return match robot_name {
        "urdf-test" | "all-4-export" => SimRobotMount::rep103_z_up_at_table_end(),
        _ => SimRobotMount::competition_placed(),
    };
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::SimRobotMount;
    use crate::robot::urdf::UrdfModel;

    #[test]
    fn rep103_mount_puts_base_on_table_and_arm_toward_plus_y() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 자산이 없습니다: {}",
            path.display()
        );
        let urdf =
            UrdfModel::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load urdf-test");
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

        let frame = crate::defaults::rail_frame();
        assert!(
            (base[2] - frame.mount_z()).abs() < 0.01,
            "base z≈프로파일 마운트: {}",
            base[2]
        );
        assert!(
            (base[1] - frame.mount_y()).abs() < 0.01,
            "base y≈프로파일 마운트: {}",
            base[1]
        );
        assert!(
            ee.position.coords.y > base[1],
            "팔 +y(테이블): base_y={} ee_y={}",
            base[1],
            ee.position.coords.y
        );
        assert!(
            ee.position.coords.z >= base[2] - 0.02,
            "베이스가 마운트보다 많이 아래로 꺼지지 않음: base_z={} ee_z={}",
            base[2],
            ee.position.coords.z
        );
    }

    /// 마운트의 y·z가 **기구학**(레일 마운트 지점)까지 반영되는지 회귀 검증.
    ///
    /// 이전에는 `arm_from_urdf::to_arm`이 레일을 primitive 템플릿에서 통째로
    /// 복사해 `SimRobotMount`의 y·z가 뷰어 배치에만 쓰이고 FK/IK에는 전혀
    /// 반영되지 않았다 — 마운트를 ±1m 옮겨도 `swing_feasibility` 결과가
    /// 완전히 동일했다(2026-07-23). 마운트 위치 튜닝 자체가 불가능한 상태였다.
    #[test]
    fn mount_position_reaches_arm_kinematics_not_just_the_viewer() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/4-dof/urdf/all-4-export.urdf");
        let arm_for = |base_y: f64, height_offset_m: f64| {
            let mut urdf =
                UrdfModel::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load 4-dof");
            urdf.mount =
                SimRobotMount::rep103_z_up_at_table_end_with_mount(base_y, height_offset_m);
            return urdf
                .to_arm(crate::defaults::DYNAMIXEL_MAX_JOINT_SPEED_RAD_S)
                .expect("Arm 변환");
        };

        let baseline = arm_for(0.02, 0.0);
        let moved = arm_for(0.12, 0.05);

        let rail_x = 0.5;
        let baseline_mount = baseline.mount_at_rail(rail_x);
        let moved_mount = moved.mount_at_rail(rail_x);

        assert!(
            (moved_mount.coords.y - baseline_mount.coords.y - 0.10).abs() < 1e-9,
            "마운트 y 이동(+0.10)이 레일 마운트 지점에 반영돼야 함: {} -> {}",
            baseline_mount.coords.y,
            moved_mount.coords.y
        );
        assert!(
            (moved_mount.coords.z - baseline_mount.coords.z - 0.05).abs() < 1e-9,
            "마운트 높이 오프셋(+0.05)이 레일 마운트 지점에 반영돼야 함: {} -> {}",
            baseline_mount.coords.z,
            moved_mount.coords.z
        );

        // 마운트가 움직였으면 같은 관절각의 EE 위치도 같은 만큼 움직여야 한다.
        let joints = baseline.default_joints.clone();
        let baseline_ee = baseline
            .forward_kinematics_with_rail(rail_x, &joints)
            .expect("baseline FK");
        let moved_ee = moved
            .forward_kinematics_with_rail(rail_x, &joints)
            .expect("moved FK");
        assert!(
            (moved_ee.position.coords.y - baseline_ee.position.coords.y - 0.10).abs() < 1e-9
                && (moved_ee.position.coords.z - baseline_ee.position.coords.z - 0.05).abs() < 1e-9,
            "EE가 마운트 이동량만큼 따라가야 함: {:?} -> {:?}",
            baseline_ee.position.coords,
            moved_ee.position.coords
        );
    }

    #[test]
    fn all_4_export_mount_points_arm_toward_table() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/4-dof/urdf/all-4-export.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 자산이 없습니다: {}",
            path.display()
        );
        let urdf = UrdfModel::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load 4-dof");
        assert_eq!(urdf.mount.rpy, [0.0, 0.0, 0.0]);
        assert_eq!(urdf.name, "all-4-export");

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

        let frame = crate::defaults::rail_frame();
        assert!(
            (base[2] - frame.mount_z()).abs() < 0.01,
            "base z≈프로파일 마운트: {}",
            base[2]
        );
        assert!(
            (base[1] - frame.mount_y()).abs() < 0.01,
            "base y≈프로파일 마운트: {}",
            base[1]
        );
        assert!(
            ee.position.coords.y > base[1],
            "4-dof 팔이 테이블(+y)로: base_y={} ee_y={}",
            base[1],
            ee.position.coords.y
        );
    }
}
