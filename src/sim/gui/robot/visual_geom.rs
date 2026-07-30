//! kiss3d 로봇 비주얼 기하 헬퍼.

use kiss3d::prelude::*;
use rapier3d::prelude::{Rotation, Vector};

use crate::robot::urdf::UrdfLinkVisual;
use crate::sim::gui::viewer::mesh_loader;

pub(crate) fn add_urdf_visual(scene: &mut SceneNode3d, vis: &UrdfLinkVisual) -> SceneNode3d {
    let color = Color::new(vis.color[0], vis.color[1], vis.color[2], vis.color[3]);
    return mesh_loader::add_geometry(scene, &vis.geometry, color);
}

pub(crate) fn urdf_link_tint(link_name: &str) -> [f32; 4] {
    let name = link_name.to_ascii_lowercase();
    if name.contains("paddle") || (name.contains("racket") && !name.contains("joint")) {
        return [0.85, 0.15, 0.12, 1.0];
    }
    if name.contains("racket_joint") {
        return [0.55, 0.55, 0.58, 1.0];
    }
    if name.contains("mx-") || name.contains("mx_") || name.contains("dynamixel") {
        return [0.95, 0.85, 0.1, 1.0];
    }
    if name == "base_link" || name.starts_with("base") {
        return [0.2, 0.25, 0.55, 1.0];
    }
    if name.contains("arm") {
        return [0.25, 0.45, 0.85, 1.0];
    }
    if name.contains("fr0") || name.contains("horn") || name.contains("bracket") {
        return [0.35, 0.55, 0.9, 1.0];
    }
    return [0.45, 0.55, 0.75, 1.0];
}

pub(crate) fn iso_from_pos_quat(pos: [f64; 3], quat_wxyz: [f64; 4]) -> nalgebra::Isometry3<f64> {
    use nalgebra::{Isometry3, Quaternion, UnitQuaternion, Vector3};
    let t = Vector3::new(pos[0], pos[1], pos[2]);
    let q = UnitQuaternion::new_normalize(Quaternion::new(
        quat_wxyz[0],
        quat_wxyz[1],
        quat_wxyz[2],
        quat_wxyz[3],
    ));
    return Isometry3::from_parts(t.into(), q);
}

pub(crate) fn rpy_xyz_to_pos(xyz: &[f64; 3]) -> Vec3 {
    return Vec3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
}

pub(crate) fn rpy_to_quat(rpy: [f64; 3]) -> Quat {
    let iso = iso_from_pos_quat([0.0, 0.0, 0.0], {
        let roll = rpy[0];
        let pitch = rpy[1];
        let yaw = rpy[2];
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    });
    let q = iso.rotation.quaternion();
    return Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32);
}

pub(crate) fn place_link(node: &mut SceneNode3d, from: Vec3, to: Vec3, radius: f32) {
    let dir = to - from;
    let length = dir.length().max(1e-4);
    let mid = (from + to) * 0.5;
    node.set_position(mid);
    if dir.length_squared() > 1e-8 {
        let axis = dir.normalize();
        let quat = Quat::from_rotation_arc(Vec3::Y, axis);
        node.set_rotation(quat);
    }
    let diameter = radius * 2.0;
    node.set_local_scale(diameter, length, diameter);
}

pub(crate) fn racket_disc_world_rotation(orientation: Rotation) -> Quat {
    let disc = Quat::from_rotation_arc(Vec3::Y, Vec3::Z);
    return to_quat(orientation) * disc;
}

pub(crate) fn to_vec3(v: Vector) -> Vec3 {
    return Vec3::new(v.x, v.y, v.z);
}

pub(crate) fn to_quat(r: Rotation) -> Quat {
    return Quat::from_xyzw(r.x, r.y, r.z, r.w);
}
