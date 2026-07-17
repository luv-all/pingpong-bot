//! 중력/항력 등 비행 물리.

use nalgebra::Vector3;

/// 중력 가속도 [m/s^2] (Z-up).
pub const G: Vector3<f64> = Vector3::new(0.0, 0.0, -9.81);

/// 중력 z 성분 [m/s^2] - 스칼라만 필요할 때.
pub const G_Z: f64 = -9.81;

/// 기본 공기저항 계수 k (측정 전 추정).
pub const DEFAULT_DRAG: f64 = 0.01;
