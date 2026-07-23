//! 랠리 임팩트·리턴 휴리스틱.

use anyhow::{Result, ensure};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpactParams {
    pub net_clearance: f64,
    pub rally_time_to_bounce: f64,
    pub racket_effective_restitution: f64,
    pub max_return_speed: f64,
}

impl ImpactParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.net_clearance >= 0.0, "net_clearance >= 0");
        ensure!(self.rally_time_to_bounce > 0.0, "rally_time_to_bounce > 0");
        ensure!(
            self.racket_effective_restitution > 0.0,
            "racket_effective_restitution > 0"
        );
        ensure!(self.max_return_speed > 0.0, "max_return_speed > 0");
        return Ok(());
    }
}

pub fn impact() -> ImpactParams {
    return ImpactParams {
        net_clearance: 0.08,
        rally_time_to_bounce: 0.55,
        racket_effective_restitution: 0.42,
        max_return_speed: 6.0,
    };
}
