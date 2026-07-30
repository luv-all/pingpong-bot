//! 테이블 위 롤 구간 (마찰 디버그용).

use crate::Point3;

#[derive(Debug, Clone)]
pub struct RollEvent {
    pub i0: usize,
    pub i1: usize,
    pub p0: Point3,
    pub p1: Point3,
    pub vt_in: f64,
    pub vt_out: f64,
    /// μ = 1 - vt_out / vt_in
    pub mu: f64,
}
