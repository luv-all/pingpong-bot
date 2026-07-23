//! sim·제어용 로봇 조립 — primitive `Arm` 또는 URDF mesh 로봇을 한 경로로 빌드한다.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::Arm;
use thiserror::Error;

use crate::robot::urdf::{SimRobotMount, UrdfLoadError, UrdfModel};

/// 제어용 `Arm` + (선택) URDF 메시·마운트.
///
/// - 단순 빌더 → `arm`만 (`urdf = None`)
/// - URDF 빌더 → `to_arm()`으로 만든 `arm` + 원본 `UrdfModel`
#[derive(Debug, Clone)]
pub struct Robot {
    /// plan_swing·관절 추종용 FK
    pub arm: Arc<Arm>,
    /// mesh 뷰어·URDF FK (없으면 primitive 렌더)
    pub urdf: Option<Arc<UrdfModel>>,
}

impl Robot {
    /// URDF 없이 primitive `Arm`만 가진 로봇.
    pub fn from_arm(arm: Arm) -> Self {
        return Self {
            arm: Arc::new(arm),
            urdf: None,
        };
    }

    /// 이미 `Arc`인 `Arm`으로 조립 (URDF 없음).
    pub fn from_shared_arm(arm: Arc<Arm>) -> Self {
        return Self { arm, urdf: None };
    }
}

/// sim 배치 프리셋.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountPreset {
    /// 내장 competition primitive / mesh 없는 URDF
    Competition,
    /// REP-103 Z-up URDF — [`crate::defaults::rail_frame`] 마운트
    Rep103AtTableEnd,
}

/// [`Robot`] 조립 빌더.
#[derive(Debug, Clone)]
pub struct RobotBuilder {
    urdf_path: Option<PathBuf>,
    ee_link: Option<String>,
    mount: Option<SimRobotMount>,
    mount_preset: Option<MountPreset>,
    max_joint_speed: f64,
}

/// [`RobotBuilder::build`] 실패.
#[derive(Debug, Error)]
pub enum RobotBuildError {
    #[error("URDF 경로가 지정되지 않았습니다")]
    MissingUrdfPath,
    #[error(transparent)]
    Urdf(#[from] UrdfLoadError),
    #[error("`Arm` 변환 실패: {reason}")]
    ArmConversion { reason: String },
}

impl RobotBuilder {
    pub fn new() -> Self {
        return Self {
            urdf_path: None,
            ee_link: None,
            mount: None,
            mount_preset: None,
            max_joint_speed: 2.5,
        };
    }

    /// URDF 파일 경로.
    pub fn urdf(mut self, path: impl AsRef<Path>) -> Self {
        self.urdf_path = Some(path.as_ref().to_path_buf());
        return self;
    }

    /// 엔드이펙터 link (`None`이면 URDF 체인 끝).
    pub fn ee_link(mut self, link: impl Into<String>) -> Self {
        self.ee_link = Some(link.into());
        return self;
    }

    pub fn ee_link_opt(mut self, link: Option<&str>) -> Self {
        if let Some(name) = link {
            self.ee_link = Some(name.to_string());
        }
        return self;
    }

    /// sim 마운트를 직접 지정 (프리셋보다 우선).
    pub fn mount(mut self, mount: SimRobotMount) -> Self {
        self.mount = Some(mount);
        return self;
    }

    /// 마운트 프리셋 (`mount()` 미지정 시 적용).
    pub fn mount_preset(mut self, preset: MountPreset) -> Self {
        self.mount_preset = Some(preset);
        return self;
    }

    /// 베이스 위치·RPY [rad]로 마운트 설정.
    pub fn mount_xyz_rpy(mut self, position: [f64; 3], rpy: [f64; 3]) -> Self {
        self.mount = Some(SimRobotMount { position, rpy });
        return self;
    }

    /// 관절 추종 최대 각속도 [rad/s].
    pub fn max_joint_speed(mut self, speed: f64) -> Self {
        self.max_joint_speed = speed;
        return self;
    }

    fn resolve_mount(&self) -> SimRobotMount {
        if let Some(mount) = self.mount {
            return mount;
        }
        let preset = self.mount_preset.unwrap_or(MountPreset::Competition);
        return match preset {
            MountPreset::Competition => SimRobotMount::competition_placed(),
            MountPreset::Rep103AtTableEnd => SimRobotMount::rep103_z_up_at_table_end(),
        };
    }

    /// primitive 전용 — fallback [`Robot`]으로 조립.
    pub fn build_primitive(fallback: Robot) -> Robot {
        return fallback;
    }

    /// URDF를 읽어 `Arm` + `UrdfModel`을 조립한다.
    pub fn build(self) -> Result<Robot, RobotBuildError> {
        let path = self
            .urdf_path
            .as_ref()
            .ok_or(RobotBuildError::MissingUrdfPath)?;

        let ee = self.ee_link.as_deref();
        let mut urdf = UrdfModel::from_file(path, ee)?;
        urdf.mount = self.resolve_mount();

        let arm =
            urdf.to_arm(self.max_joint_speed)
                .map_err(|e| RobotBuildError::ArmConversion {
                    reason: e.to_string(),
                })?;

        return Ok(Robot {
            arm: Arc::new(arm),
            urdf: Some(Arc::new(urdf)),
        });
    }
}

impl Default for RobotBuilder {
    fn default() -> Self {
        return Self::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_fallback_robot() -> Robot {
        return crate::defaults::primitive_4dof().expect("test arm");
    }

    #[test]
    fn primitive_builds_from_explicit_arm() {
        let robot = RobotBuilder::build_primitive(test_fallback_robot());
        assert!(robot.urdf.is_none());
        assert_eq!(robot.arm.joint_count(), 4);
    }

    #[test]
    fn urdf_test_builds_with_mesh() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 자산이 없습니다: {}",
            path.display()
        );
        let robot = RobotBuilder::new()
            .urdf(&path)
            .ee_link("pingpong_paddle_v5_1")
            .mount_preset(MountPreset::Rep103AtTableEnd)
            .build()
            .expect("urdf-test");
        assert!(robot.urdf.is_some());
        assert_eq!(robot.arm.joint_count(), 3);
        assert_eq!(robot.urdf.as_ref().unwrap().joint_count(), 3);
    }

    #[test]
    fn all_4_export_builds_with_mesh() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/4-dof/urdf/all-4-export.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 파일이 없습니다: {}",
            path.display()
        );
        let robot = RobotBuilder::new()
            .urdf(&path)
            .ee_link("pingpong_paddle_v5_1")
            .mount_preset(MountPreset::Rep103AtTableEnd)
            .build()
            .expect("4-dof");
        assert!(robot.urdf.is_some());
        assert_eq!(robot.arm.joint_count(), 4);
        assert_eq!(robot.urdf.as_ref().unwrap().joint_count(), 4);
    }
}
