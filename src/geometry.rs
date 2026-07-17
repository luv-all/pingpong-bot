//! 여러 기능 모듈이 공유하는 월드 좌표 기하 타입.

use nalgebra::Vector3;

/// 월드 좌표 점 [m].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub v: Vector3<f64>,
}

impl Point3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        return Self {
            v: Vector3::new(x, y, z),
        };
    }

    pub fn from_vector(v: Vector3<f64>) -> Self {
        return Self { v };
    }
}
