//! 시합용 주황 공 비주얼 (`constants::ball::RADIUS`).

use kiss3d::prelude::*;

use crate::Point3;
use crate::constants::ball;
use crate::sim::gui::scene::HIDDEN;

/// 시합용 주황 공 비주얼 (`ball::RADIUS`).
pub struct Visual {
    node: SceneNode3d,
    base_color: Color,
}

impl Visual {
    /// 시합용 주황 톤으로 공을 씬에 추가한다 (초기 위치: [`HIDDEN`]).
    pub fn spawn(scene: &mut SceneNode3d) -> Self {
        let base_color = Color::new(0.92, 0.48, 0.12, 1.0);
        return Self::spawn_with_color(scene, base_color);
    }

    /// 도달점 미리보기용 반투명 홀로그램 공.
    pub fn spawn_ghost(scene: &mut SceneNode3d) -> Self {
        let base_color = Color::new(0.35, 0.95, 1.0, 0.38);
        return Self::spawn_with_color(scene, base_color);
    }

    fn spawn_with_color(scene: &mut SceneNode3d, base_color: Color) -> Self {
        let node = scene
            .add_sphere(ball::RADIUS as f32)
            .set_color(base_color)
            .set_position(HIDDEN);
        return Self { node, base_color };
    }

    /// 월드 좌표 [m]로 공을 옮긴다.
    pub fn set_world_position(&mut self, p: Point3) {
        self.node
            .set_visible(true)
            .set_position(Vec3::new(p.x as f32, p.y as f32, p.z as f32));
    }

    /// 화면 밖으로 숨긴다.
    pub fn hide(&mut self) {
        self.node.set_visible(false).set_position(HIDDEN);
    }

    /// 기본(주황) 색.
    pub fn base_color(&self) -> Color {
        return self.base_color;
    }

    /// 임시 색 변경 (예: net-gate 실패 톤). [`restore_color`]로 되돌린다.
    pub fn set_color(&mut self, color: Color) {
        self.node.set_color(color);
    }

    /// [`base_color`]로 복구.
    pub fn restore_color(&mut self) {
        self.node.set_color(self.base_color);
    }

    /// kiss3d 노드 (고급 동기화용).
    pub fn node_mut(&mut self) -> &mut SceneNode3d {
        return &mut self.node;
    }
}
