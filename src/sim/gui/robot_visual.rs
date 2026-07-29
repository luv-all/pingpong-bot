//! 로봇 kiss3d 노드 — 경량 호스트·jog용.
//!
//! URDF가 있으면 4-dof mesh, 없으면 primitive stick-figure.

use kiss3d::prelude::*;

use super::scene::HIDDEN;
use super::viewer::mesh_loader;
use crate::constants::geometry::{
    ARM_BASE_HEIGHT, ARM_BASE_RADIUS, JOINT_MARKER_RADIUS, LINK_FOREARM_RADIUS, LINK_UPPER_RADIUS,
    RACKET_BLADE_RADIUS, RACKET_HALF_Z, RACKET_HANDLE_RADIUS,
};
use crate::robot::urdf::{UrdfLinkVisual, UrdfModel};
use crate::sim::physics::world::SimWorld;
use rapier3d::prelude::{Rotation, Vector};

/// 월드에 맞춰 그리는 로봇 비주얼.
pub enum RobotVisual {
    Primitive(PrimitiveRobotNodes),
    Urdf(UrdfRobotNodes),
}

impl RobotVisual {
    pub fn spawn(scene: &mut SceneNode3d, urdf: Option<&UrdfModel>) -> Self {
        return match urdf {
            Some(model) => Self::Urdf(UrdfRobotNodes::spawn(scene, model)),
            None => Self::Primitive(PrimitiveRobotNodes::spawn(scene)),
        };
    }

    pub fn sync_from_world(&mut self, world: &SimWorld, urdf: Option<&UrdfModel>) {
        match (self, urdf) {
            (Self::Primitive(nodes), _) => nodes.sync_from_world(world),
            (Self::Urdf(nodes), Some(model)) => nodes.sync_from_world(world, model),
            _ => {}
        }
    }
}

/// Primitive 암·라켓 노드 묶음.
pub struct PrimitiveRobotNodes {
    racket: SceneNode3d,
    racket_handle: SceneNode3d,
    arm_base: SceneNode3d,
    links: Vec<SceneNode3d>,
    link_radii: Vec<f32>,
    joints: Vec<SceneNode3d>,
    link_color: Color,
    joint_color: Color,
}

impl PrimitiveRobotNodes {
    pub fn spawn(scene: &mut SceneNode3d) -> Self {
        let link_color = Color::new(0.25, 0.45, 0.85, 1.0);
        let joint_color = Color::new(0.95, 0.85, 0.1, 1.0);
        let link_radii = [
            LINK_UPPER_RADIUS,
            LINK_UPPER_RADIUS,
            LINK_FOREARM_RADIUS,
            LINK_FOREARM_RADIUS,
            LINK_FOREARM_RADIUS,
        ];
        let links = link_radii
            .iter()
            .map(|&radius| scene.add_cylinder(radius as f32, 1.0).set_color(link_color))
            .collect();
        let joints = (0..link_radii.len())
            .map(|_| {
                scene
                    .add_sphere(JOINT_MARKER_RADIUS as f32)
                    .set_color(joint_color)
            })
            .collect();

        return Self {
            racket: scene
                .add_cylinder(RACKET_BLADE_RADIUS as f32, (RACKET_HALF_Z * 2.0) as f32)
                .set_color(Color::new(0.85, 0.15, 0.12, 1.0)),
            racket_handle: scene
                .add_cylinder(RACKET_HANDLE_RADIUS as f32, 1.0)
                .set_color(Color::new(0.55, 0.55, 0.58, 1.0)),
            arm_base: scene
                .add_cylinder(ARM_BASE_RADIUS as f32, ARM_BASE_HEIGHT as f32)
                .set_color(Color::new(0.2, 0.25, 0.55, 1.0)),
            links,
            link_radii: link_radii.iter().map(|&r| r as f32).collect(),
            joints,
            link_color,
            joint_color,
        };
    }

    pub fn sync_from_world(&mut self, world: &SimWorld) {
        let (rk_pos, rk_rot) = world.racket_pose();
        let blade = to_vec3(rk_pos);
        self.racket
            .set_position(blade)
            .set_rotation(racket_disc_world_rotation(rk_rot));

        let arm = world.arm();
        let joints = world.robot().joints();
        let Some(points) = arm.chain_points(world.robot().rail_x(), joints) else {
            return;
        };
        let points: Vec<Vec3> = points
            .into_iter()
            .map(|point| Vec3::new(point.x as f32, point.y as f32, point.z as f32))
            .collect();
        self.arm_base.set_position(points[0]);

        if points.len() >= 2 {
            let wrist = points[points.len() - 2];
            let to_blade = blade - wrist;
            let span = to_blade.length();
            let rim = if span > 1e-4 {
                let dir = to_blade / span;
                let inset = (RACKET_BLADE_RADIUS as f32).min(span * 0.95);
                blade - dir * inset
            } else {
                blade
            };
            place_link(
                &mut self.racket_handle,
                wrist,
                rim,
                RACKET_HANDLE_RADIUS as f32,
            );
        } else {
            self.racket_handle.set_position(HIDDEN);
        }

        for (index, (link, joint)) in self
            .links
            .iter_mut()
            .zip(self.joints.iter_mut())
            .enumerate()
        {
            let Some((&from, &to)) = points.get(index).zip(points.get(index + 1)) else {
                link.set_position(HIDDEN);
                joint.set_position(HIDDEN);
                continue;
            };
            joint.set_color(self.joint_color);
            link.set_color(self.link_color);
            if index + 1 == points.len() - 1 {
                link.set_position(HIDDEN);
                joint.set_position(from);
                continue;
            }
            joint.set_position(to);
            let radius = self.link_radii.get(index).copied().unwrap_or(0.015);
            place_link(link, from, to, radius);
        }
    }
}

/// URDF mesh 링크 노드.
pub struct UrdfRobotNodes {
    links: Vec<UrdfVisualNode>,
}

struct UrdfVisualNode {
    link_name: String,
    local_pos: Vec3,
    local_rot: Quat,
    node: SceneNode3d,
    base_color: Color,
}

impl UrdfRobotNodes {
    pub fn spawn(scene: &mut SceneNode3d, urdf: &UrdfModel) -> Self {
        let links = urdf
            .link_visuals()
            .into_iter()
            .map(|mut vis| {
                let rgba = urdf_link_tint(&vis.link_name);
                vis.color = rgba;
                let base_color = Color::new(rgba[0], rgba[1], rgba[2], rgba[3]);
                UrdfVisualNode {
                    link_name: vis.link_name.clone(),
                    local_pos: rpy_xyz_to_pos(&vis.origin_xyz),
                    local_rot: rpy_to_quat(vis.origin_rpy),
                    node: add_urdf_visual(scene, &vis),
                    base_color,
                }
            })
            .collect();
        return Self { links };
    }

    pub fn sync_from_world(&mut self, world: &SimWorld, urdf: &UrdfModel) {
        let joints = world
            .urdf_joint_values()
            .unwrap_or_else(|| world.robot().joints().values.clone());
        let mount = world.effective_sim_mount();
        let poses: std::collections::HashMap<String, ([f64; 3], [f64; 4])> = urdf
            .link_poses_with_mount(&joints, mount)
            .into_iter()
            .map(|(name, pos, quat)| (name, (pos, quat)))
            .collect();

        for entry in self.links.iter_mut() {
            let Some((link_pos, link_quat)) = poses.get(&entry.link_name) else {
                continue;
            };
            let link_tf = iso_from_pos_quat(*link_pos, *link_quat);
            let local_tf = iso_from_pos_quat(
                [
                    entry.local_pos.x as f64,
                    entry.local_pos.y as f64,
                    entry.local_pos.z as f64,
                ],
                [
                    entry.local_rot.w as f64,
                    entry.local_rot.x as f64,
                    entry.local_rot.y as f64,
                    entry.local_rot.z as f64,
                ],
            );
            let world_tf = link_tf * local_tf;
            let t = world_tf.translation.vector;
            let q = world_tf.rotation.quaternion();
            entry
                .node
                .set_position(Vec3::new(t.x as f32, t.y as f32, t.z as f32))
                .set_rotation(Quat::from_xyzw(
                    q.i as f32, q.j as f32, q.k as f32, q.w as f32,
                ))
                .set_color(entry.base_color);
        }
    }
}

fn add_urdf_visual(scene: &mut SceneNode3d, vis: &UrdfLinkVisual) -> SceneNode3d {
    let color = Color::new(vis.color[0], vis.color[1], vis.color[2], vis.color[3]);
    return mesh_loader::add_geometry(scene, &vis.geometry, color);
}

fn urdf_link_tint(link_name: &str) -> [f32; 4] {
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

fn iso_from_pos_quat(pos: [f64; 3], quat_wxyz: [f64; 4]) -> nalgebra::Isometry3<f64> {
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

fn rpy_xyz_to_pos(xyz: &[f64; 3]) -> Vec3 {
    return Vec3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
}

fn rpy_to_quat(rpy: [f64; 3]) -> Quat {
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

fn place_link(node: &mut SceneNode3d, from: Vec3, to: Vec3, radius: f32) {
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

fn racket_disc_world_rotation(orientation: Rotation) -> Quat {
    let disc = Quat::from_rotation_arc(Vec3::Y, Vec3::Z);
    return to_quat(orientation) * disc;
}

fn to_vec3(v: Vector) -> Vec3 {
    return Vec3::new(v.x, v.y, v.z);
}

fn to_quat(r: Rotation) -> Quat {
    return Quat::from_xyzw(r.x, r.y, r.z, r.w);
}
