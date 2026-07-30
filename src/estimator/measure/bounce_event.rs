//! 바운스 한 번 (반발계수 디버그용).

use nalgebra::Vector3;

use crate::Point3;

#[derive(Debug, Clone)]
pub struct BounceEvent {
    /// 접촉에 가까운 샘플 인덱스
    pub index: usize,
    pub contact: Point3,
    pub prev: Point3,
    pub next: Point3,
    pub v_in: Vector3<f64>,
    pub v_out: Vector3<f64>,
    /// e = |vz_out| / |vz_in|
    pub e: f64,
}
