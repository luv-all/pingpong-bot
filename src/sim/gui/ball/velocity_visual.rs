//! 공 속도 벡터 화살표.

use kiss3d::prelude::*;

use crate::Point3;
use crate::sim::gui::scene::HIDDEN;

/// 화살대 지름 [m]. `set_local_scale`은 절대 치수를 받으므로 반지름의 2배.
const SHAFT_DIAMETER: f32 = 0.012;
/// 화살대 초기 길이 [m] (매 프레임 `set_local_scale`로 갱신).
const SHAFT_LEN_INIT: f32 = 0.2;

/// 공 속도 벡터 화살표 (jog 홀로그램 공 오버레이).
pub struct VelocityVisual {
    shaft: SceneNode3d,
    tip: SceneNode3d,
}

impl VelocityVisual {
    pub fn spawn(scene: &mut SceneNode3d) -> Self {
        let color = Color::new(0.35, 0.95, 1.0, 0.95);
        let shaft = scene
            .add_cylinder(SHAFT_DIAMETER * 0.5, SHAFT_LEN_INIT)
            .set_color(color)
            .set_position(HIDDEN);
        let tip = scene
            .add_cone(0.014, 0.045)
            .set_color(color)
            .set_position(HIDDEN);
        return Self { shaft, tip };
    }

    /// 시작점 + 속도 벡터를 월드에 표시.
    pub fn set_from_velocity(&mut self, origin: Point3, velocity: [f64; 3]) {
        let v = Vec3::new(velocity[0] as f32, velocity[1] as f32, velocity[2] as f32);
        let speed = v.length();
        if speed < 1e-4 {
            self.hide();
            return;
        }
        let dir = v / speed;
        let shaft_len = (speed * 0.06).clamp(0.08, 0.35);
        let tip_len = 0.045_f32;
        let origin = Vec3::new(origin.x as f32, origin.y as f32, origin.z as f32);
        let rot = Quat::from_rotation_arc(Vec3::Y, dir);
        self.shaft
            .set_visible(true)
            .set_local_scale(SHAFT_DIAMETER, shaft_len, SHAFT_DIAMETER)
            .set_position(origin + dir * (shaft_len * 0.5))
            .set_rotation(rot);
        self.tip
            .set_visible(true)
            .set_position(origin + dir * (shaft_len + tip_len * 0.5))
            .set_rotation(rot);
    }

    pub fn hide(&mut self) {
        self.shaft.set_visible(false).set_position(HIDDEN);
        self.tip.set_visible(false).set_position(HIDDEN);
    }
}
