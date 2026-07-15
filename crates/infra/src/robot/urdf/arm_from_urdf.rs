//! URDF → domain `Arm` 일반 revolute 직렬 체인 변환.

use nalgebra::{Isometry3, Vector3};
use pingpong_domain::{Arm, SerialChain, SerialJoint};
use urdf_rs::JointType;

use super::{UrdfLoadError, UrdfRobot, fk};

pub fn try_into_arm(urdf: &UrdfRobot, max_joint_speed: f64) -> Result<Arm, UrdfLoadError> {
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
                pending = Isometry3::identity();
            }
            JointType::Fixed => {}
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

    let mount = urdf.mount.isometry();
    let chain = SerialChain::new(mount.rotation, joints, pending).map_err(|e| {
        UrdfLoadError::ArmConversion {
            reason: e.to_string(),
        }
    })?;
    return Arm::from_serial_chain(
        pingpong_domain::Point3::from_vector(mount.translation.vector),
        Some(rail),
        chain,
        limits,
        defaults,
        max_joint_speed,
    )
    .map_err(|e| UrdfLoadError::ArmConversion {
        reason: format!("{e}"),
    });
}
