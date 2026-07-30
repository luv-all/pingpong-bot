//! [`super::guidance::step_racket_guidance`] 한 스텝 결과.

use nalgebra::Vector3;

/// [`step_racket_guidance`] 한 스텝의 결과 — 호출부가 진단/리포트에 쓴다.
pub struct RacketGuidanceStep {
    pub racket_accel_desired: Vector3<f64>,
    pub torque_cmd: Vec<f64>,
}
