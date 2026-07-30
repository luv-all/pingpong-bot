//! `--sim-verify` 관절별 결과 한 줄.

use serde::Serialize;

/// `--sim-verify` 관절별 결과 한 줄.
#[derive(Debug, Serialize)]
pub struct ContactVerifyJointRow {
    pub joint: usize,
    pub tracking_error_at_contact_rad: Option<f64>,
    pub tracking_error_at_planned_impact_rad: Option<f64>,
    pub peak_commanded_speed_rad_s: f64,
}
