//! 임팩트 목표 자세·속도.

use nalgebra::Vector3;
use pingpong_bot::robot::Joints;

pub struct Target {
    pub rail_x: f64,
    pub joints: Joints,
    pub rail_velocity: f64,
    pub joint_velocities: Vec<f64>,
    /// 임팩트 순간 라켓이 실제로 내야 하는 속도(월드).
    pub racket_velocity: Vector3<f64>,
}
