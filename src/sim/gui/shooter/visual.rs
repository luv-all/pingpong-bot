//! 슈터 본체 비주얼 (직육면체, 충돌 없음 — 표시 전용).

use kiss3d::prelude::*;
use rapier3d::prelude::{Rotation, Vector};

use crate::sim::launch;

/// 슈터 본체 직육면체. 발사구가 전면에 오도록 조준축 뒤로 물려 그린다.
pub struct Visual {
    node: SceneNode3d,
}

impl Visual {
    pub fn spawn(scene: &mut SceneNode3d) -> Self {
        let node = scene
            .add_cube(
                launch::Layout::VISUAL_SIZE_X as f32,
                launch::Layout::VISUAL_SIZE_Y as f32,
                launch::Layout::VISUAL_SIZE_Z as f32,
            )
            .set_color(Color::new(0.45, 0.45, 0.5, 1.0));
        return Self { node };
    }

    /// 물리 월드가 준 본체 자세 그대로 (`SimWorld::shooter_pose`).
    pub fn set_pose(&mut self, position: Vector, rotation: Rotation) {
        self.node
            .set_position(Vec3::new(position.x, position.y, position.z))
            .set_rotation(Quat::from_xyzw(
                rotation.x, rotation.y, rotation.z, rotation.w,
            ));
    }

    /// 설정에서 직접 — 월드 없이 그릴 때. `SimWorld::sync_shooter_pose`와 같은 SSOT.
    pub fn set_from_settings(&mut self, settings: &launch::Settings) {
        self.set_pose(settings.visual_position(), settings.orientation());
    }

    pub fn node_mut(&mut self) -> &mut SceneNode3d {
        return &mut self.node;
    }
}
