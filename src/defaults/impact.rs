//! 랠리 임팩트·리턴 휴리스틱.

use anyhow::{Result, ensure};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpactParams {
    pub net_clearance: f64,
    pub rally_time_to_bounce: f64,
    pub racket_effective_restitution: f64,
    /// Rapier 라켓 collider 접선 마찰.
    pub racket_friction: f64,
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
        ensure!(
            (0.0..=1.0).contains(&self.racket_friction),
            "racket_friction in 0..=1"
        );
        ensure!(self.max_return_speed > 0.0, "max_return_speed > 0");
        return Ok(());
    }
}

impl Default for ImpactParams {
    fn default() -> Self {
        return Self {
            net_clearance: 0.08,
            rally_time_to_bounce: 0.55,
            racket_effective_restitution: 0.55,
            racket_friction: 0.5,
            max_return_speed: 6.0,
        };
    }
}

