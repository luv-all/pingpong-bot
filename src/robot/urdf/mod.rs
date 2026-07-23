//! URDF 로봇 모델 로드·순기구학.

mod arm_from_urdf;
mod fk;
mod mount;
mod visual;

use std::path::{Path, PathBuf};

use crate::{Arm, JointLimit, Joints, RacketPose};
use thiserror::Error;
use urdf_rs::{JointType, Robot};

pub use mount::SimRobotMount;
pub use visual::{UrdfGeometry, UrdfLinkVisual};

/// 파싱된 URDF 모델 — sim 뷰어 메시·마운트·(선택) URDF FK.
#[derive(Debug, Clone)]
pub struct UrdfModel {
    /// URDF `<robot name="...">`
    pub name: String,
    /// URDF 파일 기준 디렉터리 (`package://`·상대 mesh 경로 해석용)
    pub base_dir: PathBuf,
    /// 엔드이펙터 link 이름
    pub ee_link: String,
    /// sim 월드 배치 (REP-103 Z-up → sim Z-up)
    pub mount: SimRobotMount,
    robot: Robot,
    /// root → ee 순서의 actuated revolute/continuous 관절 인덱스
    actuated_chain: Vec<usize>,
}

/// URDF 로드 실패.
#[derive(Debug, Error)]
pub enum UrdfLoadError {
    #[error("URDF 파일 읽기 실패: {path}")]
    Read {
        path: PathBuf,
        #[source]
        source: urdf_rs::UrdfError,
    },
    #[error("엔드이펙터 link `{link}` 를 URDF에서 찾을 수 없습니다")]
    EndEffectorNotFound { link: String },
    #[error("link `{link}` 까지의 관절 체인을 구성할 수 없습니다")]
    ChainNotFound { link: String },
    #[error("actuated revolute 관절이 없습니다 (ee={ee_link})")]
    NoActuatedJoints { ee_link: String },
    #[error("`Arm` 변환 실패: {reason}")]
    ArmConversion { reason: String },
}

impl UrdfModel {
    /// URDF 파일을 읽는다. `ee_link`가 `None`이면 마지막 child link를 사용한다.
    pub fn from_file(path: impl AsRef<Path>, ee_link: Option<&str>) -> Result<Self, UrdfLoadError> {
        let path = path.as_ref();
        let base_dir = path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));

        let robot = urdf_rs::read_file(path).map_err(|source| UrdfLoadError::Read {
            path: path.to_path_buf(),
            source,
        })?;

        let ee = match ee_link {
            Some(name) => name.to_string(),
            None => default_ee_link(&robot).ok_or_else(|| UrdfLoadError::EndEffectorNotFound {
                link: "(auto)".to_string(),
            })?,
        };

        if !robot.links.iter().any(|l| l.name == ee) {
            return Err(UrdfLoadError::EndEffectorNotFound { link: ee });
        }

        let chain = fk::chain_joint_indices(&robot, &ee)
            .ok_or(UrdfLoadError::ChainNotFound { link: ee.clone() })?;

        let actuated_chain: Vec<usize> = chain
            .into_iter()
            .filter(|&idx| {
                matches!(
                    robot.joints[idx].joint_type,
                    JointType::Revolute | JointType::Continuous
                )
            })
            .collect();

        if actuated_chain.is_empty() {
            return Err(UrdfLoadError::NoActuatedJoints { ee_link: ee });
        }

        return Ok(Self {
            name: robot.name.clone(),
            base_dir,
            ee_link: ee,
            mount: mount::default_sim_mount(&robot.name),
            robot,
            actuated_chain,
        });
    }

    /// revolute/continuous 관절 수.
    pub fn joint_count(&self) -> usize {
        return self.actuated_chain.len();
    }

    /// 관절 이름 (actuated 순서).
    pub fn joint_names(&self) -> Vec<String> {
        return self
            .actuated_chain
            .iter()
            .map(|&idx| self.robot.joints[idx].name.clone())
            .collect();
    }

    /// 기본 관절각 [rad] — URDF limit 중점 또는 0.
    pub fn default_joints(&self) -> Joints {
        let values = self
            .actuated_chain
            .iter()
            .map(|&idx| default_joint_angle(&self.robot.joints[idx]))
            .collect();
        return Joints { values };
    }

    /// 관절 한계 (actuated 순서).
    pub fn joint_limits(&self) -> Vec<Option<JointLimit>> {
        return self
            .actuated_chain
            .iter()
            .map(|&idx| limit_from_joint(&self.robot.joints[idx]))
            .collect();
    }

    /// kiss3d용 link visual 목록.
    pub fn link_visuals(&self) -> Vec<UrdfLinkVisual> {
        return visual::collect_link_visuals(&self.robot, |uri| self.resolve_path(uri));
    }

    /// URDF FK + [`Self::mount`] (뷰어·물리 배치).
    pub fn link_poses_in_sim(&self, joints: &[f64]) -> Vec<(String, [f64; 3], [f64; 4])> {
        return fk::link_world_poses_in_sim(
            &self.robot,
            joints,
            &self.actuated_chain,
            self.mount.isometry(),
        );
    }

    /// 엔드이펙터 `RacketPose` (URDF 로컬).
    pub fn end_effector_pose(&self, joints: &[f64]) -> Option<RacketPose> {
        return fk::end_effector_pose(&self.robot, &self.ee_link, joints, &self.actuated_chain);
    }

    /// 엔드이펙터 `RacketPose` + [`Self::mount`].
    pub fn end_effector_pose_in_sim(&self, joints: &[f64]) -> Option<RacketPose> {
        return fk::end_effector_pose_in_sim(
            &self.robot,
            &self.ee_link,
            joints,
            &self.actuated_chain,
            self.mount.isometry(),
        );
    }

    /// link pose + 임의 마운트 (테스트·튜닝용).
    pub fn link_poses_with_mount(
        &self,
        joints: &[f64],
        mount: SimRobotMount,
    ) -> Vec<(String, [f64; 3], [f64; 4])> {
        return fk::link_world_poses_in_sim(
            &self.robot,
            joints,
            &self.actuated_chain,
            mount.isometry(),
        );
    }

    /// URDF actuated 체인을 제어 `Arm`으로 변환.
    pub fn to_arm(&self, max_joint_speed: f64) -> Result<Arm, UrdfLoadError> {
        return arm_from_urdf::to_arm(self, max_joint_speed);
    }

    /// mesh·텍스처 상대 경로를 URDF 기준 디렉터리로 해석한다.
    pub fn resolve_path(&self, uri: &str) -> PathBuf {
        if let Some(rest) = uri.strip_prefix("package://") {
            let parts: Vec<&str> = rest.split('/').collect();
            let pkg = parts.first().copied().unwrap_or("");
            let rel = parts.get(1..).map(|s| s.join("/")).unwrap_or_default();

            // ROS 패키지 레이아웃: `{pkg}/urdf/*.urdf` + `{pkg}/meshes/*.stl`
            if let Some(parent) = self.base_dir.parent() {
                if parent.file_name().and_then(|n| n.to_str()) == Some(pkg) {
                    return parent.join(&rel);
                }
            }

            return self.base_dir.join(&rel);
        }
        if uri.starts_with("file://") {
            return PathBuf::from(uri.trim_start_matches("file://"));
        }
        let path = Path::new(uri);
        if path.is_absolute() {
            return path.to_path_buf();
        }
        return self.base_dir.join(path);
    }
}

fn default_ee_link(robot: &Robot) -> Option<String> {
    let child_links: std::collections::HashSet<_> =
        robot.joints.iter().map(|j| j.child.link.as_str()).collect();
    let roots: Vec<_> = robot
        .links
        .iter()
        .filter(|l| !child_links.contains(l.name.as_str()))
        .collect();
    if roots.len() == 1 {
        let mut tip = roots[0].name.clone();
        loop {
            let next = robot
                .joints
                .iter()
                .find(|j| j.parent.link == tip)
                .map(|j| j.child.link.clone());
            match next {
                Some(child) => tip = child,
                None => return Some(tip),
            }
        }
    }
    return robot.links.last().map(|l| l.name.clone());
}

fn default_joint_angle(joint: &urdf_rs::Joint) -> f64 {
    let limit = &joint.limit;
    if limit.lower < limit.upper {
        return (limit.lower + limit.upper) * 0.5;
    }
    return 0.0;
}

fn limit_from_joint(joint: &urdf_rs::Joint) -> Option<JointLimit> {
    if joint.joint_type == JointType::Continuous {
        return None;
    }
    let lower = joint.limit.lower;
    let upper = joint.limit.upper;
    if lower < upper {
        return Some(JointLimit::new(lower, upper));
    }
    return None;
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_URDF: &str = r#"<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="racket">
    <visual>
      <geometry><box size="0.16 0.18 0.012"/></geometry>
    </visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.2" upper="1.2" effort="10" velocity="2.5"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.35 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.2" upper="1.4" effort="10" velocity="2.5"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="link2"/>
    <child link="racket"/>
    <origin xyz="0.30 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="0.5" effort="10" velocity="2.5"/>
  </joint>
</robot>"#;

    #[test]
    fn loads_urdf_test_robot_with_package_meshes() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 자산이 없습니다: {}",
            path.display()
        );

        let urdf =
            UrdfModel::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load urdf-test");
        assert_eq!(urdf.name, "urdf-test");
        assert_eq!(urdf.joint_count(), 3);
        assert_eq!(
            urdf.joint_names(),
            ["Revolute 6", "Revolute 9", "Revolute 13"]
        );

        let mesh = urdf.resolve_path("package://urdf-test_description/meshes/base_link.stl");
        assert!(
            mesh.exists(),
            "ROS package mesh 경로 해석 실패: {}",
            mesh.display()
        );

        let arm = urdf.to_arm(2.5).expect("3축 URDF Arm 변환");
        assert_eq!(arm.joint_count(), urdf.joint_count());
        let joints = urdf.default_joints();
        let expected = urdf
            .end_effector_pose_in_sim(&joints.values)
            .expect("URDF FK");
        let actual = arm.forward_kinematics(&joints).expect("domain FK");
        assert!((actual.position.coords - expected.position.coords).norm() < 1e-9);
        assert!((actual.normal - expected.normal).norm() < 1e-9);
        assert_eq!(
            arm.with_wrist_open(&joints, 0.7).expect("3축 no-op"),
            joints,
            "별도 손목이 없는 3축 체인의 위치 관절을 racket-open으로 덮지 않아야 함"
        );
    }

    #[test]
    fn loads_4dof_export_with_package_meshes() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/4-dof/urdf/all-4-export.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 자산이 없습니다: {}",
            path.display()
        );

        let urdf = UrdfModel::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load 4-dof");
        assert_eq!(urdf.name, "all-4-export");
        assert_eq!(urdf.joint_count(), 4);
        assert_eq!(
            urdf.joint_names(),
            ["Revolute 6", "Revolute 9", "Revolute 13", "Revolute 18"]
        );

        for vis in urdf.link_visuals() {
            if let UrdfGeometry::Mesh { path, .. } = &vis.geometry {
                assert!(
                    path.exists(),
                    "mesh 미존재 (절대경로·package 해석 확인): {}",
                    path.display()
                );
            }
        }

        // URDF에서 만든 domain Arm은 원본 URDF의 FK와 정확히 같아야 한다.
        let arm = urdf.to_arm(2.5).expect("4축 Arm 변환");
        assert_eq!(arm.joint_count(), 4);
        assert_eq!(
            arm.joint_limit(0),
            None,
            "continuous 축은 가짜 한계가 없어야 함"
        );
        for values in [
            urdf.default_joints().values,
            vec![0.1, 0.4, -0.3, 0.2],
            vec![-0.2, 0.8, 0.15, -0.4],
        ] {
            let expected = urdf.end_effector_pose_in_sim(&values).expect("URDF FK");
            let actual = arm
                .forward_kinematics(&Joints { values })
                .expect("domain FK");
            assert!(
                (actual.position.coords - expected.position.coords).norm() < 1e-9,
                "URDF/domain EE position mismatch: actual={:?} expected={:?}",
                actual.position,
                expected.position
            );
            assert!(
                (actual.normal - expected.normal).norm() < 1e-9,
                "URDF/domain EE normal mismatch: actual={:?} expected={:?}",
                actual.normal,
                expected.normal
            );
            let [w, x, y, z] = actual.orientation;
            let orientation =
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z));
            assert!(
                (orientation * nalgebra::Vector3::z() - actual.normal).norm() < 1e-9,
                "domain 라켓 local +Z가 면 법선이어야 함"
            );
        }

        let hint = urdf.default_joints();
        let mut target_joints = hint.clone();
        for (index, value) in target_joints.values.iter_mut().take(3).enumerate() {
            let offset = *value + 0.08;
            *value = arm
                .joint_limit(index)
                .map_or(offset, |limit| offset.clamp(limit.min, limit.max));
        }
        let target = arm
            .forward_kinematics(&target_joints)
            .expect("target FK")
            .position;
        let solved = arm
            .inverse_kinematics_near(target, Some(&hint))
            .expect("URDF 수치 IK");
        let solved_pose = arm.forward_kinematics(&solved).expect("solved FK");
        assert!((solved_pose.position.coords - target.coords).norm() < 1e-5);
    }

    #[test]
    fn competition_primitive_matches_simplified_4dof_urdf_chain() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/4-dof/urdf/all-4-export.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 자산이 없습니다: {}",
            path.display()
        );

        let urdf = UrdfModel::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load 4-dof");
        let primitive = crate::defaults::arm().expect("competition primitive");
        for values in [
            vec![0.0, 0.0, -0.25, 0.0],
            vec![0.15, 0.2, -0.4, 0.35],
            vec![-0.3, -0.2, 0.5, -0.45],
        ] {
            let expected = urdf.end_effector_pose_in_sim(&values).expect("URDF FK");
            let actual = primitive
                .forward_kinematics(&Joints {
                    values: values.clone(),
                })
                .expect("primitive FK");
            assert!(
                (actual.position.coords - expected.position.coords).norm() < 1e-9,
                "values={values:?}, actual={:?}, expected={:?}",
                actual.position,
                expected.position
            );
            assert!((actual.normal - expected.normal).norm() < 1e-9);
        }
    }

    #[test]
    fn loads_urdf_from_string_via_tempfile() {
        let dir = std::env::temp_dir().join("pingpong_bot_urdf_test");
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("arm.urdf");
        std::fs::write(&path, SAMPLE_URDF).expect("write urdf");

        let urdf = UrdfModel::from_file(&path, Some("racket")).expect("load");
        assert_eq!(urdf.joint_count(), 3);
        assert_eq!(urdf.ee_link, "racket");
        assert!(urdf.end_effector_pose(&[0.0, 0.5, -0.3]).is_some());
    }
}
