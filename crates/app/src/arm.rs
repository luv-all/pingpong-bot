//! 배포·실험별 로봇 스펙 (plan §2).
//!
//! **어떤 로봇을 쓸지**는 app이 정한다 — domain 타입만 사용, URDF 로드는 infra/bin.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use pingpong_domain::{Arm, ArmBuildError, constants::table};

/// sim에서 URDF 루트를 올릴 위치·자세 (infra `SimRobotMount`와 동일 의미).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotMount {
    /// 월드 위치 [m]
    pub position: [f64; 3],
    /// 고정축 RPY [rad]
    pub rpy: [f64; 3],
}

impl RobotMount {
    /// 내장 primitive / competition URDF용 — 리니어 레일 원점 (x=0).
    pub const fn competition_table_end() -> Self {
        return Self {
            position: [0.0, 0.02, table::SURFACE_Z],
            rpy: [0.0, 0.0, 0.0],
        };
    }

    /// REP-103 mesh URDF (`urdf-test` 등) — Z-up sim, 탁구대 y≈0 끝.
    pub const fn rep103_table_end() -> Self {
        return Self::competition_table_end();
    }
}

/// 내장 URDF 로봇 — `assets/robots/urdf-test/`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UrdfTestRobot;

impl UrdfTestRobot {
    /// 워크스페이스 루트 기준 상대 경로.
    pub const URDF_REL: &'static str =
        "assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf";
    pub const EE_LINK: &'static str = "pingpong_paddle_v5_1";
    pub const MAX_JOINT_SPEED: f64 = 2.5;

    pub const fn mount() -> RobotMount {
        return RobotMount::rep103_table_end();
    }

    pub fn urdf_path(workspace_root: impl AsRef<Path>) -> PathBuf {
        return workspace_root.as_ref().join(Self::URDF_REL);
    }
}

/// 내장 competition URDF (`assets/robots/competition_arm.urdf`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompetitionUrdfRobot;

impl CompetitionUrdfRobot {
    pub const URDF_REL: &'static str = "assets/robots/competition_arm.urdf";
    pub const EE_LINK: &'static str = "racket_link";
    pub const MAX_JOINT_SPEED: f64 = 2.5;

    pub const fn mount() -> RobotMount {
        return RobotMount::competition_table_end();
    }

    pub fn urdf_path(workspace_root: impl AsRef<Path>) -> PathBuf {
        return workspace_root.as_ref().join(Self::URDF_REL);
    }
}

/// 런타임에 선택하는 로봇 배포.
#[derive(Debug, Clone, PartialEq)]
pub enum Robot {
    /// URDF 없이 domain `Arm` primitive만 (기본).
    CompetitionPrimitive,
    /// `UrdfTestRobot` mesh 로봇.
    UrdfTest,
    /// `--urdf` CLI 지정.
    CustomUrdf {
        path: PathBuf,
        ee_link: Option<String>,
        mount: RobotMount,
        max_joint_speed: f64,
    },
}

impl Robot {
    /// CLI `--urdf` / `--ee-link` 로 배포를 고른다.
    pub fn from_cli(urdf: Option<PathBuf>, ee_link: Option<String>) -> Self {
        return match urdf {
            None => Self::CompetitionPrimitive,
            Some(path) => Self::CustomUrdf {
                path,
                ee_link,
                mount: RobotMount::rep103_table_end(),
                max_joint_speed: 2.5,
            },
        };
    }

    pub fn is_primitive(&self) -> bool {
        return matches!(self, Self::CompetitionPrimitive);
    }

    /// URDF 파일 경로 (`CompetitionPrimitive`면 `None`).
    pub fn urdf_path(&self, workspace_root: impl AsRef<Path>) -> Option<PathBuf> {
        return match self {
            Self::CompetitionPrimitive => None,
            Self::UrdfTest => Some(UrdfTestRobot::urdf_path(workspace_root)),
            Self::CustomUrdf { path, .. } => Some(path.clone()),
        };
    }

    pub fn ee_link(&self) -> Option<&str> {
        return match self {
            Self::CompetitionPrimitive => None,
            Self::UrdfTest => Some(UrdfTestRobot::EE_LINK),
            Self::CustomUrdf { ee_link, .. } => ee_link.as_deref(),
        };
    }

    pub fn mount(&self) -> RobotMount {
        return match self {
            Self::CompetitionPrimitive => RobotMount::competition_table_end(),
            Self::UrdfTest => UrdfTestRobot::mount(),
            Self::CustomUrdf { mount, .. } => *mount,
        };
    }

    pub fn max_joint_speed(&self) -> f64 {
        return match self {
            Self::CompetitionPrimitive => 2.5,
            Self::UrdfTest => UrdfTestRobot::MAX_JOINT_SPEED,
            Self::CustomUrdf {
                max_joint_speed, ..
            } => *max_joint_speed,
        };
    }
}

/// GIST 경진용 3DOF primitive `Arm` + X축 리니어 레일.
pub fn competition_arm() -> Result<Arm, ArmBuildError> {
    return Arm::competition();
}

/// 파이프라인·sim에서 공유하는 primitive `Arm`.
pub fn shared_competition_arm() -> Arc<Arm> {
    return Arc::new(Arm::competition().expect("경진용 arm은 항상 유효해야 합니다"));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn competition_arm_builds() {
        let arm = competition_arm().expect("프리셋");
        assert_eq!(arm.joint_count(), 3);
    }

    #[test]
    fn default_robot_is_primitive() {
        let robot = Robot::from_cli(None, None);
        assert!(robot.is_primitive());
        assert!(robot.urdf_path(".").is_none());
    }

    #[test]
    fn urdf_test_has_builtin_path() {
        let robot = Robot::UrdfTest;
        let path = robot.urdf_path(".").expect("path");
        assert!(path.ends_with("urdf-test.urdf"));
    }
}
