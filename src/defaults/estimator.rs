//! 궤적 추정(EKF·탄도) 휴리스틱.

use anyhow::{Result, ensure};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorParams {
    pub min_lead: f64,
    pub max_lead: f64,
    pub integrate_dt: f64,
    pub min_approach_speed_y: f64,
    pub min_strike_clearance: f64,
    pub q_pos: f64,
    pub q_vel: f64,
    pub r_meas: f64,
}

impl EstimatorParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.min_lead > 0.0, "min_lead > 0");
        ensure!(self.max_lead >= self.min_lead, "max_lead >= min_lead");
        ensure!(self.integrate_dt > 0.0, "integrate_dt > 0");
        ensure!(self.min_approach_speed_y > 0.0, "min_approach_speed_y > 0");
        ensure!(self.min_strike_clearance >= 0.0, "min_strike_clearance >= 0");
        ensure!(self.q_pos >= 0.0, "q_pos >= 0");
        ensure!(self.q_vel >= 0.0, "q_vel >= 0");
        ensure!(self.r_meas > 0.0, "r_meas > 0");
        return Ok(());
    }
}

pub fn estimator() -> EstimatorParams {
    return EstimatorParams {
        min_lead: 0.05,
        max_lead: 1.2,
        integrate_dt: 0.001,
        min_approach_speed_y: 0.8,
        min_strike_clearance: 0.05,
        q_pos: 1.0e-4,
        q_vel: 1.0e-2,
        r_meas: 0.0009,
    };
}
