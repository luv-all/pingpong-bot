//! 스윕 후보 집계.

use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct CandidateResult {
    pub base_y: f64,
    pub mount_base_z_m: f64,
    pub speed_mps: f64,
    pub pitch_deg: f64,
    pub height_offset_m: f64,
    pub shots: usize,
    pub incoming_valid: usize,
    pub committed: usize,
    pub contact: usize,
    pub returned: usize,
    pub cleared_net: usize,
    pub success: usize,
    pub returned_in: usize,
    pub best_peak_ratio: f64,
    pub median_peak_ratio: f64,
}
