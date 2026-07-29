//! URDF mesh 링크 노드.

use kiss3d::prelude::*;

use crate::robot::urdf::UrdfModel;
use crate::robot::urdf_visual_node::UrdfVisualNode;
use crate::robot::visual_geom::{
    add_urdf_visual, iso_from_pos_quat, rpy_to_quat, rpy_xyz_to_pos, urdf_link_tint,
};
use crate::sim::physics::world::SimWorld;

/// URDF mesh 링크 노드.
pub struct UrdfNodes {
    links: Vec<UrdfVisualNode>,
}

impl UrdfNodes {
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
