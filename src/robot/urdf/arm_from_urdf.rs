//! URDF → domain `Arm` 일반 revolute 직렬 체인 변환.

use crate::constants::control::{CONTINUOUS_TORQUE_DERATE, MX28_STALL_TORQUE_NM, MX64_STALL_TORQUE_NM};
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
                // 4-DOF 로봇의 알려진 관절 인덱스→모터 매핑(joint0=yaw,
                // joint1=shoulder=MX-64R / joint2=elbow, joint3=wrist=MX-28T,
                // 근거: `.omc/research/dynamixel-specs.md`)을 실제 모터 스펙
                // 기반 토크 한계로 쓴다 — `Arm::competition()`과 동일 SSOT.
                // URDF의 `<limit effort>`(예: "100")는 CAD 익스포터 기본값이라
                // 실제 모터 정격과 무관해 쓰지 않는다. yaw(continuous)는 URDF에
                // `<limit>` 태그 자체가 없어 effort가 0(→ 이전엔 무한대 폴백)
                // 이었는데, 이 역시 실제로는 같은 MX-64R이라 무한대가 아니다.
                // joint0(yaw)은 모터 2배 — URDF의 `Rigid 4`/`Rigid 5`가
                // `base_link`에 MX-64R 두 대를 대칭 고정하는데, `Revolute 6`
                // (yaw)은 그중 하나만 부모로 삼아 운동학적으로는 관절 1개지만
                // 실기에서 두 모터가 기계적으로 결합돼 같은 축에 토크를 함께
                // 낸다(2026-07-23, 하드웨어 담당자 확인, `Arm::competition()`과
                // 동일 근거). 4관절을 벗어나는(향후) URDF만 옛
                // effort-or-무한대로 되돌아간다.
                let motor_derived_limit = match joint_torque_limits.len() {
                    0 => Some(2.0 * MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE),
                    1 => Some(MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE),
                    2 | 3 => Some(MX28_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE),
                    _ => None,
                };
                let effort = joint.limit.effort;
                joint_torque_limits.push(
                    motor_derived_limit
                        .unwrap_or_else(|| if effort > 0.0 { effort } else { f64::INFINITY }),
                );
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
