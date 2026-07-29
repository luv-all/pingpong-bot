use std::f64::consts::TAU;

use super::dynamixel_config::DynamixelConfig;
use super::dynamixel_config_error::DynamixelConfigError;

/// URDF 관절각과 Dynamixel 절대 tick 사이의 순수 좌표 변환.
#[derive(Debug, Clone)]
pub struct MotorMapping {
    pub(super) config: DynamixelConfig,
    tick_limits: Vec<(i32, i32)>,
}

impl MotorMapping {
    pub fn new(config: DynamixelConfig) -> Result<Self, DynamixelConfigError> {
        config.validate()?;
        let tick_limits = config
            .motor_angle_limits_deg
            .iter()
            .map(|[lo, hi]| {
                let to_tick = |degrees: f64| {
                    (degrees * f64::from(config.ticks_per_revolution) / 360.0).round() as i32
                };
                (to_tick(*lo), to_tick(*hi))
            })
            .collect();
        return Ok(Self {
            config,
            tick_limits,
        });
    }

    pub fn config(&self) -> &DynamixelConfig {
        return &self.config;
    }

    pub fn radians_to_ticks(&self, joint_index: usize, angle_rad: f64) -> i32 {
        let sign = f64::from(self.config.joint_signs[joint_index]);
        let adjusted = sign * angle_rad + self.config.joint_offsets_rad[joint_index];
        let ticks = (f64::from(self.config.zero_tick)
            + adjusted * f64::from(self.config.ticks_per_revolution) / TAU)
            .round() as i32;
        let (lo, hi) = self.tick_limits[joint_index];
        return ticks.clamp(lo, hi);
    }

    pub fn ticks_to_radians(&self, joint_index: usize, ticks: i32) -> f64 {
        let raw = f64::from(ticks - self.config.zero_tick) * TAU
            / f64::from(self.config.ticks_per_revolution);
        let sign = f64::from(self.config.joint_signs[joint_index]);
        return sign * (raw - self.config.joint_offsets_rad[joint_index]);
    }
}
