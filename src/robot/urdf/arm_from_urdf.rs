//! URDF → domain `Arm` 일반 revolute 직렬 체인 변환.

use crate::{Arm, LinkInertial, SerialChain, SerialJoint};
use nalgebra::{Isometry3, Matrix3, Vector3};
use urdf_rs::JointType;

use super::{UrdfLoadError, UrdfRobot, fk};

pub fn to_arm(urdf: &UrdfRobot, max_joint_speed: f64) -> Result<Arm, UrdfLoadError> {
    let defaults = urdf.default_joints();
    let limits = urdf.joint_limits();
    let template = Arm::competition().map_err(|e| UrdfLoadError::ArmConversion {
        reason: format!("레일·베이스 템플릿: {e}"),
    })?;
    let rail = template.rail.expect("competition arm은 레일 포함");
    let full_chain = fk::chain_joint_indices(&urdf.robot, &urdf.ee_link).ok_or_else(|| {
        UrdfLoadError::ArmConversion {
            reason: format!("root에서 EE `{}`까지 체인을 찾을 수 없습니다", urdf.ee_link),
        }
    })?;

    let mut pending = Isometry3::identity();
    let mut joints = Vec::with_capacity(urdf.joint_count());
    let mut link_inertials = Vec::with_capacity(urdf.joint_count());
    let mut aggregated_inertials = Vec::with_capacity(urdf.joint_count());
    let mut joint_torque_limits = Vec::with_capacity(urdf.joint_count());
    // 현재 revolute 관절이 움직이는 강체를 합성하기 위한 하위 링크 누적기.
    // `pending`은 이전 revolute의 child link 프레임을 기준(identity)으로 하므로
    // fixed joint를 만날 때의 `pending` 값이 그 하위 링크의 배치 변환이 된다.
    let mut current_agg: Option<Vec<(Isometry3<f64>, LinkInertial)>> = None;
    let find_link = |name: &str, joint_name: &str| {
        urdf.robot
            .links
            .iter()
            .find(|link| link.name == name)
            .ok_or_else(|| UrdfLoadError::ArmConversion {
                reason: format!("관절 `{joint_name}`의 child link `{name}`를 찾을 수 없습니다"),
            })
    };
    for joint_index in full_chain {
        let joint = &urdf.robot.joints[joint_index];
        pending *= fk::pose_to_iso(&joint.origin);
        match joint.joint_type {
            JointType::Revolute | JointType::Continuous => {
                let axis = Vector3::new(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]);
                joints.push(SerialJoint::new(pending, axis).map_err(|e| {
                    UrdfLoadError::ArmConversion {
                        reason: format!("관절 `{}`: {e}", joint.name),
                    }
                })?);
                let child_link = find_link(&joint.child.link, &joint.name)?;
                let child_inertial = link_inertial_from_urdf(child_link);
                link_inertials.push(child_inertial);
                // 이전 revolute의 강체 합성을 마무리하고 새 관절 합성을 시작한다.
                if let Some(bodies) = current_agg.take() {
                    aggregated_inertials.push(LinkInertial::combine(&bodies));
                }
                current_agg = Some(vec![(Isometry3::identity(), child_inertial)]);
                // URDF `<limit effort>`를 토크 한계로 쓴다 (effort<=0이면 무제한).
                // competition() primitive와 달리 URDF에는 모터 모델이 인코딩돼
                // 있지 않아, URDF가 명시한 effort를 관절 토크 상한으로 삼는다.
                let effort = joint.limit.effort;
                joint_torque_limits.push(if effort > 0.0 { effort } else { f64::INFINITY });
                pending = Isometry3::identity();
            }
            JointType::Fixed => {
                // revolute child에 fixed로 붙은 하위 링크 — 현재 관절 강체에 합친다.
                // (첫 revolute 이전의 fixed 링크는 base 소속이라 누적기가 없다.)
                if let Some(bodies) = current_agg.as_mut() {
                    let child_link = find_link(&joint.child.link, &joint.name)?;
                    bodies.push((pending, link_inertial_from_urdf(child_link)));
                }
            }
            _ => {
                return Err(UrdfLoadError::ArmConversion {
                    reason: format!(
                        "관절 `{}` 타입 {:?}은 아직 제어 체인에서 지원하지 않습니다",
                        joint.name, joint.joint_type
                    ),
                });
            }
        }
    }
    // 마지막 revolute 관절(손목)의 강체 합성 마무리 — EE 쪽 fixed 링크(패들 등) 포함.
    if let Some(bodies) = current_agg.take() {
        aggregated_inertials.push(LinkInertial::combine(&bodies));
    }

    let mount = urdf.mount.isometry();
    let chain = SerialChain::new(mount.rotation, joints, pending).map_err(|e| {
        UrdfLoadError::ArmConversion {
            reason: e.to_string(),
        }
    })?;
    return Arm::from_serial_chain(
        crate::Point3::from(mount.translation.vector),
        Some(rail),
        chain,
        limits,
        link_inertials,
        aggregated_inertials,
        joint_torque_limits,
        defaults,
        max_joint_speed,
    )
    .map_err(|e| UrdfLoadError::ArmConversion {
        reason: format!("{e}"),
    });
}

/// URDF link의 `<inertial>` (질량/원점/텐서)를 그대로 domain `LinkInertial`로 옮긴다.
fn link_inertial_from_urdf(link: &urdf_rs::Link) -> LinkInertial {
    let inertial = &link.inertial;
    let origin = &inertial.origin.xyz;
    let tensor = &inertial.inertia;
    return LinkInertial {
        mass: inertial.mass.value,
        com: crate::Point3::new(origin[0], origin[1], origin[2]),
        inertia: Matrix3::new(
            tensor.ixx, tensor.ixy, tensor.ixz, //
            tensor.ixy, tensor.iyy, tensor.iyz, //
            tensor.ixz, tensor.iyz, tensor.izz,
        ),
    };
}
