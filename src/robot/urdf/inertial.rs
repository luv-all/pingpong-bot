//! URDF 링크 관성을 revolute 체인 순서로 합산한다.

use nalgebra::{Isometry3, Matrix3, Vector3};
use urdf_rs::{JointType, Robot};

use super::fk::{chain_joint_indices, pose_to_iso};
use super::{UrdfLoadError, UrdfModel};
use crate::robot::dynamics::LinkInertia;

/// EE 체인 기준, actuated revolute마다 하나 — fixed 자식은 child 프레임으로 합산.
pub fn link_inertias_for_chain(urdf: &UrdfModel) -> Result<Vec<LinkInertia>, UrdfLoadError> {
    let full = chain_joint_indices(&urdf.robot, &urdf.ee_link).ok_or_else(|| {
        UrdfLoadError::ChainNotFound {
            link: urdf.ee_link.clone(),
        }
    })?;
    let mut out = Vec::with_capacity(urdf.joint_count());
    let mut i = 0;
    while i < full.len() {
        let joint_idx = full[i];
        let joint = &urdf.robot.joints[joint_idx];
        if !matches!(
            joint.joint_type,
            JointType::Revolute | JointType::Continuous
        ) {
            i += 1;
            continue;
        }
        let child_name = joint.child.link.clone();
        let mut composite = link_inertia(&urdf.robot, &child_name)?;
        let mut t_child_in_first = Isometry3::identity();
        let mut j = i + 1;
        while j < full.len() {
            let next = &urdf.robot.joints[full[j]];
            if matches!(
                next.joint_type,
                JointType::Revolute | JointType::Continuous
            ) {
                break;
            }
            // fixed (또는 기타 비구동) — child를 first-child 프레임으로 합산
            let t_joint = pose_to_iso(&next.origin);
            t_child_in_first *= t_joint;
            let fixed_child = link_inertia(&urdf.robot, &next.child.link)?;
            composite = composite.combine(&fixed_child, &t_child_in_first);
            j += 1;
        }
        out.push(composite);
        i = j;
    }
    if out.len() != urdf.joint_count() {
        return Err(UrdfLoadError::ArmConversion {
            reason: format!(
                "관성 링크 수 {} ≠ actuated {}",
                out.len(),
                urdf.joint_count()
            ),
        });
    }
    return Ok(out);
}

fn link_inertia(robot: &Robot, name: &str) -> Result<LinkInertia, UrdfLoadError> {
    let link = robot
        .links
        .iter()
        .find(|l| l.name == name)
        .ok_or_else(|| UrdfLoadError::ArmConversion {
            reason: format!("link `{name}` 없음"),
        })?;
    let inertial = &link.inertial;
    let mass = inertial.mass.value;
    let com = Vector3::new(
        inertial.origin.xyz[0],
        inertial.origin.xyz[1],
        inertial.origin.xyz[2],
    );
    // URDF inertia는 COM 기준, link 축 정렬 (origin rpy는 보통 0).
    let i = &inertial.inertia;
    let mut inertia_com = Matrix3::zeros();
    inertia_com[(0, 0)] = i.ixx;
    inertia_com[(0, 1)] = i.ixy;
    inertia_com[(0, 2)] = i.ixz;
    inertia_com[(1, 0)] = i.ixy;
    inertia_com[(1, 1)] = i.iyy;
    inertia_com[(1, 2)] = i.iyz;
    inertia_com[(2, 0)] = i.ixz;
    inertia_com[(2, 1)] = i.iyz;
    inertia_com[(2, 2)] = i.izz;
    if inertial.origin.rpy.iter().any(|&v| v.abs() > 1e-12) {
        let r = pose_to_iso(&inertial.origin).rotation.to_rotation_matrix();
        inertia_com = r * inertia_com * r.transpose();
    }
    return Ok(LinkInertia {
        mass,
        com,
        inertia_com,
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn four_dof_inertias_match_actuated_count_and_positive_mass() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/4-dof/urdf/all-4-export.urdf");
        let urdf = UrdfModel::from_file(&path, Some("pingpong_paddle_v5_1")).expect("load");
        let inertias = link_inertias_for_chain(&urdf).expect("inertias");
        assert_eq!(inertias.len(), 4);
        for (i, link) in inertias.iter().enumerate() {
            assert!(
                link.mass > 0.05,
                "link {i} mass too small after fixed merge: {}",
                link.mass
            );
        }
        // 첫 링크: FR05-H101 + FR05-B101 + MX-64R ≈ 0.052+0.019+0.126
        assert!((inertias[0].mass - 0.19678).abs() < 1e-3);
    }
}
