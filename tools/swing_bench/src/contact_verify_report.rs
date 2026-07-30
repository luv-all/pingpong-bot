//! `--sim-verify` 결과.

use serde::Serialize;

use crate::contact_verify_joint_row::ContactVerifyJointRow;

/// `--sim-verify` 결과.
#[derive(Debug, Serialize)]
pub struct ContactVerifyReport {
    pub swing_committed: bool,
    pub contact_detected: bool,
    pub planned_impact_time_secs: f64,
    pub contact_elapsed_secs: Option<f64>,
    pub contact_vs_planned_delta_secs: Option<f64>,
    pub joints: Vec<ContactVerifyJointRow>,
}
