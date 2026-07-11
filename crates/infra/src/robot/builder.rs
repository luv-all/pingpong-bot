//! sim·제어용 로봇 조립 — primitive `Arm` 또는 URDF mesh 로봇을 한 경로로 빌드한다.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use pingpong_domain::Arm;
use thiserror::Error;

use crate::robot::urdf::{SimRobotMount, UrdfLoadError, UrdfRobot};

/// 빌드된 sim 로봇 — 제어용 `Arm` + (선택) URDF FK·mesh 뷰어.
#[derive(Debug, Clone)]
pub struct SimRobot {
    /// plan_swing·관절 추종용 (4축 FK)
    pub arm: Arc<Arm>,
    /// mesh 뷰어·URDF FK (없으면 primitive 렌더)
    pub urdf: Option<Arc<UrdfRobot>>,
}

/// sim 배치 프리셋.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountPreset {
    /// 내장 competition_arm / mesh 없는 URDF
    Competition,
    /// REP-103 Z-up URDF — 탁구대 y≈0 끝
    Rep103AtTableEnd,
}

/// [`SimRobot`] 조립 빌더.
#[derive(Debug, Clone)]
pub struct RobotBuilder {
    urdf_path: Option<PathBuf>,
    ee_link: Option<String>,
    mount: Option<SimRobotMount>,
    mount_preset: Option<MountPreset>,
    max_joint_speed: f64,
    use_competition_primitive: bool,
}

/// [`RobotBuilder::build`] 실패.
#[derive(Debug, Error)]
pub enum RobotBuildError {
    #[error("URDF 경로가 지정되지 않았습니다 (primitive 모드는 `competition()` 사용)")]
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
            use_competition_primitive: false,
        };
    }

    /// 내장 primitive 4DOF 팔 (URDF 없음).
    pub fn competition() -> Self {
        return Self::new().use_competition_primitive(true);
    }

    fn use_competition_primitive(mut self, value: bool) -> Self {
        self.use_competition_primitive = value;
        return self;
    }

    /// URDF 파일 경로.
    pub fn urdf(mut self, path: impl AsRef<Path>) -> Self {
        self.urdf_path = Some(path.as_ref().to_path_buf());
        self.use_competition_primitive = false;
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

    fn resolve_mount(&self, robot_name: &str) -> SimRobotMount {
        if let Some(mount) = self.mount {
            return mount;
        }
        let _ = robot_name;
        let preset = self.mount_preset.unwrap_or(MountPreset::Competition);
        return match preset {
            MountPreset::Competition => SimRobotMount::competition_placed(),
            MountPreset::Rep103AtTableEnd => SimRobotMount::rep103_z_up_at_table_end(),
        };
    }

    /// primitive 전용 — app `shared_competition_arm()` 등 fallback `Arm`으로 조립.
    pub fn build_primitive(fallback: Arc<Arm>) -> SimRobot {
        return SimRobot {
            arm: fallback,
            urdf: None,
        };
    }

    /// `SimRobot`을 조립한다 (URDF 필수).
    pub fn build(self) -> Result<SimRobot, RobotBuildError> {
        if self.use_competition_primitive {
            return Err(RobotBuildError::MissingUrdfPath);
        }

        let path = self
            .urdf_path
            .as_ref()
            .ok_or(RobotBuildError::MissingUrdfPath)?;

        let ee = self.ee_link.as_deref();
        let mut urdf = UrdfRobot::from_file(path, ee)?;
        urdf.mount = self.resolve_mount(&urdf.name);

        let arm = urdf.try_into_arm(self.max_joint_speed).map_err(|e| {
            RobotBuildError::ArmConversion {
                reason: e.to_string(),
            }
        })?;

        return Ok(SimRobot {
            arm: Arc::new(arm),
            urdf: Some(Arc::new(urdf)),
        });
    }

    /// URDF FK·뷰어는 유지하고 제어만 fallback `Arm`으로 빌드한다.
    pub fn build_with_arm_fallback(self, fallback: Arc<Arm>) -> Result<SimRobot, RobotBuildError> {
        if self.use_competition_primitive {
            return Ok(Self::build_primitive(fallback));
        }

        let path = self
            .urdf_path
            .as_ref()
            .ok_or(RobotBuildError::MissingUrdfPath)?;

        let ee = self.ee_link.as_deref();
        let mut urdf = UrdfRobot::from_file(path, ee)?;
        urdf.mount = self.resolve_mount(&urdf.name);

        let arm = match urdf.try_into_arm(self.max_joint_speed) {
            Ok(arm) => Arc::new(arm),
            Err(_) => fallback,
        };

        return Ok(SimRobot {
            arm,
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

    fn test_fallback_arm() -> Arc<Arm> {
        return Arc::new(Arm::competition().expect("test arm"));
    }

    #[test]
    fn primitive_builds_with_fallback() {
        let robot = RobotBuilder::competition()
            .build_with_arm_fallback(test_fallback_arm())
            .expect("competition");
        assert!(robot.urdf.is_none());
        assert_eq!(robot.arm.joint_count(), 4);
    }

    #[test]
    fn urdf_test_builds_with_mesh() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
        if !path.exists() {
            return;
        }
        // urdf-test mesh는 3축 — 제어 Arm은 4DOF competition fallback
        let robot = RobotBuilder::new()
            .urdf(&path)
            .ee_link("pingpong_paddle_v5_1")
            .mount_preset(MountPreset::Rep103AtTableEnd)
            .build_with_arm_fallback(test_fallback_arm())
            .expect("urdf-test");
        assert!(robot.urdf.is_some());
        assert_eq!(robot.arm.joint_count(), 4);
        assert_eq!(robot.urdf.as_ref().unwrap().joint_count(), 3);
    }
}
