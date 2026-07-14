//! URDF 순기구학 — link pose·엔드이펙터.

use std::collections::HashMap;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use pingpong_domain::types::Point3;
use pingpong_domain::{RacketPose};
use urdf_rs::{Joint, JointType, Robot};

/// root → `ee_link` 관절 인덱스 (parent→child 순).
pub fn chain_joint_indices(robot: &Robot, ee_link: &str) -> Option<Vec<usize>> {
    let mut chain = Vec::new();
    let mut current = ee_link.to_string();
    loop {
        let joint_idx = robot.joints.iter().position(|j| j.child.link == current)?;
        chain.push(joint_idx);
        current = robot.joints[joint_idx].parent.link.clone();
        if robot.joints.iter().all(|j| j.child.link != current) {
            break;
        }
    }
    chain.reverse();
    return Some(chain);
}

/// actuated 관절각으로 모든 link 월드 pose.
pub fn link_world_poses(
    robot: &Robot,
    joint_values: &[f64],
    actuated_chain: &[usize],
) -> Vec<(String, [f64; 3], [f64; 4])> {
    let transforms = link_transforms(robot, joint_values, actuated_chain);
    return transforms
        .into_iter()
        .map(|(name, iso)| iso_to_pose_tuple(&name, iso))
        .collect();
}

/// URDF FK + sim 마운트 변환.
pub fn link_world_poses_in_sim(
    robot: &Robot,
    joint_values: &[f64],
    actuated_chain: &[usize],
    mount: Isometry3<f64>,
) -> Vec<(String, [f64; 3], [f64; 4])> {
    let transforms = link_transforms(robot, joint_values, actuated_chain);
    return transforms
        .into_iter()
        .map(|(name, iso)| iso_to_pose_tuple(&name, mount * iso))
        .collect();
}

/// 엔드이펙터 link → `RacketPose` (link +x 를 면 법선으로 사용).
pub fn end_effector_pose(
    robot: &Robot,
    ee_link: &str,
    joint_values: &[f64],
    actuated_chain: &[usize],
) -> Option<RacketPose> {
    let transforms = link_transforms(robot, joint_values, actuated_chain);
    let iso = transforms.into_iter().find(|(n, _)| n == ee_link)?.1;
    return Some(racket_pose_from_iso(iso));
}

/// 엔드이펙터 `RacketPose` + sim 마운트.
pub fn end_effector_pose_in_sim(
    robot: &Robot,
    ee_link: &str,
    joint_values: &[f64],
    actuated_chain: &[usize],
    mount: Isometry3<f64>,
) -> Option<RacketPose> {
    let transforms = link_transforms(robot, joint_values, actuated_chain);
    let iso = transforms.into_iter().find(|(n, _)| n == ee_link)?.1;
    return Some(racket_pose_from_iso(mount * iso));
}

pub(crate) fn mount_to_iso(position: [f64; 3], rpy: [f64; 3]) -> Isometry3<f64> {
    let t = Vector3::new(position[0], position[1], position[2]);
    let r = rpy_to_quat(rpy[0], rpy[1], rpy[2]);
    return Isometry3::from_parts(t.into(), r);
}

fn iso_to_pose_tuple(name: &str, iso: Isometry3<f64>) -> (String, [f64; 3], [f64; 4]) {
    let t = iso.translation.vector;
    let q = iso.rotation.quaternion();
    return (
        name.to_string(),
        [t.x, t.y, t.z],
        [q.w, q.i, q.j, q.k],
    );
}

fn racket_pose_from_iso(iso: Isometry3<f64>) -> RacketPose {
    let position = Point3::new(iso.translation.x, iso.translation.y, iso.translation.z);
    let rot = iso.rotation.to_rotation_matrix();
    let normal = rot * Vector3::new(1.0, 0.0, 0.0);
    let q = iso.rotation.quaternion();
    return RacketPose {
        position,
        normal: Vector3::new(normal.x, normal.y, normal.z),
        orientation: [q.w, q.i, q.j, q.k],
    };
}

fn link_transforms(
    robot: &Robot,
    joint_values: &[f64],
    actuated_chain: &[usize],
) -> HashMap<String, Isometry3<f64>> {
    let root = find_root_link(robot);
    let mut q_iter = joint_values.iter().copied();
    let mut actuated_set: HashMap<usize, f64> = HashMap::new();
    for &j_idx in actuated_chain {
        if let Some(q) = q_iter.next() {
            actuated_set.insert(j_idx, q);
        }
    }

    let mut out = HashMap::new();
    let identity = Isometry3::identity();
    out.insert(root.clone(), identity);
    let mut queue = vec![root];
    while let Some(parent_link) = queue.pop() {
        let parent_tf = *out.get(&parent_link).unwrap_or(&identity);
        for (j_idx, joint) in robot.joints.iter().enumerate() {
            if joint.parent.link != parent_link {
                continue;
            }
            let q = actuated_set.get(&j_idx).copied().unwrap_or(0.0);
            let joint_tf = joint_transform(joint, q);
            let child_tf = parent_tf * joint_tf;
            out.insert(joint.child.link.clone(), child_tf);
            queue.push(joint.child.link.clone());
        }
    }
    return out;
}

pub(crate) fn find_root_link(robot: &Robot) -> String {
    let children: std::collections::HashSet<_> =
        robot.joints.iter().map(|j| j.child.link.as_str()).collect();
    if let Some(root) = robot
        .links
        .iter()
        .find(|l| !children.contains(l.name.as_str()))
    {
        return root.name.clone();
    }
    return robot
        .links
        .first()
        .map(|l| l.name.clone())
        .unwrap_or_default();
}

fn joint_transform(joint: &Joint, q: f64) -> Isometry3<f64> {
    let origin = pose_to_iso(&joint.origin);
    let motion = match joint.joint_type {
        JointType::Revolute | JointType::Continuous => {
            let axis = Vector3::new(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]);
            if axis.norm_squared() < 1e-12 {
                Isometry3::identity()
            } else {
                Isometry3::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), q),
                )
            }
        }
        JointType::Prismatic => {
            let axis = Vector3::new(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]);
            Isometry3::translation(axis.x * q, axis.y * q, axis.z * q)
        }
        JointType::Fixed => Isometry3::identity(),
        _ => Isometry3::identity(),
    };
    return origin * motion;
}

fn pose_to_iso(pose: &urdf_rs::Pose) -> Isometry3<f64> {
    let t = Vector3::new(pose.xyz[0], pose.xyz[1], pose.xyz[2]);
    let r = rpy_to_quat(pose.rpy[0], pose.rpy[1], pose.rpy[2]);
    return Isometry3::from_parts(t.into(), r);
}

/// URDF RPY (roll-pitch-yaw, fixed axis) → 쿼터니언.
fn rpy_to_quat(roll: f64, pitch: f64, yaw: f64) -> UnitQuaternion<f64> {
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    return UnitQuaternion::new_normalize(Quaternion::new(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_at_zero() {
        let pose = urdf_rs::Pose {
            xyz: urdf_rs::Vec3([1.0, 2.0, 3.0]),
            rpy: urdf_rs::Vec3([0.0, 0.0, 0.0]),
        };
        let iso = pose_to_iso(&pose);
        assert!((iso.translation.x - 1.0).abs() < 1e-9);
    }
}
