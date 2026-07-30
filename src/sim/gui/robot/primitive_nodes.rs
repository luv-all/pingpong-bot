//! Primitive 암·라켓 노드 묶음.

use kiss3d::prelude::*;

use super::visual_geom::{place_link, racket_disc_world_rotation, to_vec3};
use crate::constants::geometry::{
    ARM_BASE_HEIGHT, ARM_BASE_RADIUS, JOINT_MARKER_RADIUS, LINK_FOREARM_RADIUS, LINK_UPPER_RADIUS,
    RACKET_BLADE_RADIUS, RACKET_HALF_Z, RACKET_HANDLE_RADIUS,
};
use crate::sim::gui::scene::HIDDEN;
use crate::sim::physics::world::SimWorld;

/// Primitive 암·라켓 노드 묶음.
pub struct PrimitiveNodes {
    racket: SceneNode3d,
    racket_handle: SceneNode3d,
    arm_base: SceneNode3d,
    links: Vec<SceneNode3d>,
    link_radii: Vec<f32>,
    joints: Vec<SceneNode3d>,
    link_color: Color,
    joint_color: Color,
}

impl PrimitiveNodes {
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
