//! 로봇 카탈로그 — **여기만** 만진다 (id·URDF·링크 길이·관절·뷰어 매핑).
//!
//! `base_xyz`는 Arm 빌더에, URDF mesh 월드 배치는 bin이 탁구대 끝에 고정한다
//! (infra `SimRobotMount` — “마운트” = 시뮬 월드에 로봇 루트를 올리는 위치·자세).
//!
//! `control_to_urdf`: 제어 DOF ≠ URDF actuated일 때 뷰어 FK 매핑.
//! `None`이면 앞쪽 관절 truncate. 경연용 실URDF는 나중에 받고 identity로 맞춘다.

use std::path::{Path, PathBuf};
use std::sync::{Arc, LazyLock};

use pingpong_domain::{
    Arm, ArmBuildError,
    constants::{
        RACKET_OPEN_PITCH,
        arm::{
            BASE_Y, ELBOW_DEFAULT, ELBOW_MAX, ELBOW_MIN, LINK_FOREARM, LINK_UPPER, LINK_WRIST_STUB,
            MAX_JOINT_SPEED, RAIL_MAX_SPEED, SHOULDER_DEFAULT, SHOULDER_MAX, SHOULDER_MIN,
            WRIST_MAX, WRIST_MIN, YAW_DEFAULT, YAW_MAX, YAW_MIN,
},
        table,
    },
};

/// TOML / CLI 기본 id ([`ROBOTS`] 첫 항목과 맞출 것).
pub const DEFAULT_ROBOT_ID: &str = "competition";

/// urdf-test / competition_arm.urdf (3축 mesh) ← 제어 4DOF 앞 3축.
const MAP_4_TO_3: &[Option<usize>] = &[Some(0), Some(1), Some(2)];

/// 카탈로그 한 줄.
#[derive(Clone, Copy)]
pub struct RobotEntry {
    pub id: &'static str,
    /// 시각화 URDF (없으면 빌더만). 제어·IK는 항상 [`Self::build`].
    pub urdf_rel: Option<&'static str>,
    pub ee_link: Option<&'static str>,
    pub max_joint_speed: f64,
    /// 제어 인덱스 → URDF actuated 슬롯. 길이 = URDF joint_count.
    pub control_to_urdf: Option<&'static [Option<usize>]>,
    pub build: fn() -> Result<Arm, ArmBuildError>,
}

impl RobotEntry {
    pub fn arm(self) -> Arc<Arm> {
        return Arc::new((self.build)().expect("카탈로그 빌더는 유효해야 함"));
    }

    pub fn urdf_path(self, workspace_root: impl AsRef<Path>) -> Option<PathBuf> {
        return self.urdf_rel.map(|rel| workspace_root.as_ref().join(rel));
    }

    /// 뷰어용 매핑 복사 (`None` = truncate fallback).
    pub fn control_to_urdf_owned(self) -> Option<Vec<Option<usize>>> {
        return self.control_to_urdf.map(|m| m.to_vec());
    }
}

/// **로봇 SSOT**.
pub static ROBOTS: LazyLock<Vec<RobotEntry>> = LazyLock::new(|| {
    let build_competition = || {
        Arm::builder()
            .base_xyz(0.0, BASE_Y, table::SURFACE_Z)
            .linear_rail(
                BASE_Y,
                table::SURFACE_Z,
                0.0,
                table::WIDTH_X,
                RAIL_MAX_SPEED,
            )
            .link(LINK_UPPER)
            .revolute_at(YAW_MIN, YAW_MAX, YAW_DEFAULT)
            .link(LINK_FOREARM)
            .revolute_at(SHOULDER_MIN, SHOULDER_MAX, SHOULDER_DEFAULT)
            .link(LINK_WRIST_STUB)
            .revolute_at(ELBOW_MIN, ELBOW_MAX, ELBOW_DEFAULT)
            .link(LINK_WRIST_STUB)
            .revolute_at(WRIST_MIN, WRIST_MAX, RACKET_OPEN_PITCH)
            .max_joint_speed(MAX_JOINT_SPEED)
            .build()
    };

    return vec![
        RobotEntry {
            id: "competition",
            urdf_rel: None,
            ee_link: None,
            max_joint_speed: MAX_JOINT_SPEED,
            control_to_urdf: None,
            build: build_competition,
        },
        RobotEntry {
            id: "urdf-test",
            urdf_rel: Some("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf"),
            ee_link: Some("pingpong_paddle_v5_1"),
            max_joint_speed: 2.5,
            control_to_urdf: Some(MAP_4_TO_3),
            build: build_competition,
        },
        RobotEntry {
            id: "competition-urdf",
            urdf_rel: Some("assets/robots/competition_arm.urdf"),
            ee_link: Some("racket_link"),
            max_joint_speed: 2.5,
            control_to_urdf: Some(MAP_4_TO_3),
            build: build_competition,
        },
        RobotEntry {
            id: "4-dof",
            urdf_rel: Some("assets/robots/4-dof/urdf/all-4-export.urdf"),
            ee_link: Some("pingpong_paddle_v5_1"),
            max_joint_speed: 2.5,
            control_to_urdf: None,
            build: build_competition,
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
        .arm();
}

pub fn competition_arm() -> Result<Arm, ArmBuildError> {
    return (find_robot(DEFAULT_ROBOT_ID)
        .expect("DEFAULT_ROBOT_ID")
        .build)();
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
    fn urdf_test_maps_first_three_control_joints() {
        let entry = find_robot("urdf-test").unwrap();
        assert_eq!(entry.control_to_urdf, Some(MAP_4_TO_3));
        assert_eq!(entry.control_to_urdf.unwrap().len(), 3);
    }

    #[test]
    fn unknown_id() {
        assert!(find_robot("nope").is_none());
        assert!(robot_ids_csv().contains("urdf-test"));
        assert!(robot_ids_csv().contains("4-dof"));
    }
}
