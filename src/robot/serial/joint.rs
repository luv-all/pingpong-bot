//! 한 revolute 관절의 URDF 기하.

use nalgebra::{Isometry3, Unit, Vector3};

use super::SerialChainError;

/// 한 revolute 관절의 URDF 기하.
///
/// 변환 순서는 URDF와 동일하게 `origin * rotation(axis, q)`다.
#[derive(Debug, Clone, PartialEq)]
pub struct SerialJoint {
    pub origin: Isometry3<f64>,
    pub axis: Unit<Vector3<f64>>,
}

impl SerialJoint {
    pub fn new(origin: Isometry3<f64>, axis: Vector3<f64>) -> Result<Self, SerialChainError> {
        if !axis.iter().all(|v| v.is_finite()) || axis.norm_squared() < 1e-12 {
            return Err(SerialChainError::InvalidAxis);
        }
        // -0.0 제거 — 다운스트림 f32 조인트 기저가 손갈라지지 않게.
        let axis = Vector3::new(axis.x + 0.0, axis.y + 0.0, axis.z + 0.0);
        return Ok(Self {
            origin,
            axis: Unit::new_normalize(axis),
        });
    }
}
