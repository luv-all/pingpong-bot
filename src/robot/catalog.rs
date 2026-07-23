//! 로봇 카탈로그 — **여기만** 만진다 (id·URDF·EE·제어 속도).
//!
//! URDF가 있는 항목은 URDF 자체가 제어·FK·IK·뷰어의 단일 모델이다.

use std::path::{Path, PathBuf};
use std::sync::{Arc, LazyLock};

use crate::{Arm, ArmBuildError, hardware::dynamixel::DYNAMIXEL_MAX_JOINT_SPEED_RAD_S};

/// TOML / CLI 기본 id ([`ROBOTS`] 첫 항목과 맞출 것).
pub const DEFAULT_ROBOT_ID: &str = "competition";

/// 카탈로그 한 줄.
#[derive(Clone, Copy)]
pub struct RobotEntry {
    pub id: &'static str,
    /// 제어·FK·IK·시각화 URDF. `None`이면 primitive 빌더를 사용한다.
    pub urdf_rel: Option<&'static str>,
    pub ee_link: Option<&'static str>,
    pub max_joint_speed: f64,
    pub primitive: Option<fn() -> Result<Arm, ArmBuildError>>,
}

impl RobotEntry {
    pub fn primitive_arm(self) -> Option<Arc<Arm>> {
        return self
            .primitive
            .map(|build| Arc::new(build().expect("카탈로그 빌더는 유효해야 함")));
    }

    pub fn urdf_path(self, workspace_root: impl AsRef<Path>) -> Option<PathBuf> {
        return self.urdf_rel.map(|rel| workspace_root.as_ref().join(rel));
    }
}

/// **로봇 SSOT**.
pub static ROBOTS: LazyLock<Vec<RobotEntry>> = LazyLock::new(|| {
    let build_competition = Arm::competition;

    return vec![
        RobotEntry {
            id: "competition",
            urdf_rel: None,
            ee_link: None,
            max_joint_speed: DYNAMIXEL_MAX_JOINT_SPEED_RAD_S,
            primitive: Some(build_competition),
        },
        RobotEntry {
            id: "urdf-test",
            urdf_rel: Some("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf"),
            ee_link: Some("pingpong_paddle_v5_1"),
            max_joint_speed: DYNAMIXEL_MAX_JOINT_SPEED_RAD_S,
            primitive: None,
        },
        RobotEntry {
            id: "4-dof",
            urdf_rel: Some("assets/robots/4-dof/urdf/all-4-export.urdf"),
            ee_link: Some("pingpong_paddle_v5_1"),
            max_joint_speed: DYNAMIXEL_MAX_JOINT_SPEED_RAD_S,
            primitive: None,
        },
    ];
});

pub fn find_robot(id: &str) -> Option<&'static RobotEntry> {
    return ROBOTS.iter().find(|e| e.id == id);
}

pub fn robot_ids_csv() -> String {
    return ROBOTS.iter().map(|e| e.id).collect::<Vec<_>>().join(" | ");
}

pub fn shared_competition_arm() -> Arc<Arm> {
    return find_robot(DEFAULT_ROBOT_ID)
        .expect("DEFAULT_ROBOT_ID")
        .primitive_arm()
        .expect("기본 로봇은 primitive");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_builds_4dof() {
        let arm = shared_competition_arm();
        assert_eq!(arm.joint_count(), 4);
    }

    #[test]
    fn catalog_ids() {
        assert!(find_robot(DEFAULT_ROBOT_ID).is_some());
        assert_eq!(ROBOTS[0].id, DEFAULT_ROBOT_ID);
        let path = find_robot("urdf-test").unwrap().urdf_path(".").unwrap();
        assert!(path.ends_with("urdf-test.urdf"));
        let path4 = find_robot("4-dof").unwrap().urdf_path(".").unwrap();
        assert!(path4.ends_with("all-4-export.urdf"));
    }

    #[test]
    fn urdf_entries_do_not_define_parallel_primitive_models() {
        let entry = find_robot("urdf-test").unwrap();
        assert!(entry.primitive.is_none());
        assert!(entry.primitive_arm().is_none());
    }

    #[test]
    fn unknown_id() {
        assert!(find_robot("nope").is_none());
        assert!(robot_ids_csv().contains("urdf-test"));
        assert!(robot_ids_csv().contains("4-dof"));
    }

    #[test]
    fn urdf_entries_link_inertials_match_joint_count() {
        use crate::UrdfRobot;

        for id in ["urdf-test", "4-dof"] {
            let entry = find_robot(id).expect("카탈로그 항목");
            let path = entry
                .urdf_path(env!("CARGO_MANIFEST_DIR"))
                .expect("urdf 항목은 경로가 있어야 함");
            let urdf = UrdfRobot::from_file(&path, entry.ee_link).expect("URDF 로드");
            let arm = urdf.to_arm(entry.max_joint_speed).expect("Arm 변환");
            assert_eq!(
                arm.link_inertials.len(),
                arm.joint_count(),
                "id={id}: link_inertials 개수가 joint_count와 달라야 함"
            );
        }
    }
}
