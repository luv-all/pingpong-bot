//! TOML 시나리오.

use serde::Deserialize;

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub robot: Option<String>,
    pub start_rail_x: Option<f64>,
    pub impact: Option<[f64; 3]>,
    pub incoming_velocity: Option<[f64; 3]>,
    pub time_budget_secs: Option<f64>,
}
