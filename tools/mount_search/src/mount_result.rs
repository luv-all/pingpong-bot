//! 마운트 후보 채점 결과.

use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct MountResult {
    pub base_y: f64,
    /// 베이스 z [m] — 바닥(z=0) 기준 절대 좌표.
    pub base_z: f64,
    pub feasible_count: usize,
    pub total: usize,
    pub mean_peak_ratio: f64,
    pub worst_peak_ratio: f64,
}
