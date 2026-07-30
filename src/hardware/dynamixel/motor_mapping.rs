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

    /// [`Self::radians_to_ticks`]가 모터 각도 한계로 **잘랐는가**.
    ///
    /// 플래너는 URDF 관절 한계만 본다 — `motor_angle_limits_deg`는 그보다 좁을 수 있고,
    /// 여기서 조용히 잘리면 팔이 명령과 다른 곳에 선다. dry-run 클립에서 joint 3이
    /// 0.50 rad(28.8°) 어긋난 게 이것이었다: 모터가 못 따라간 게 아니라 **갈 수 없는 각도를
    /// 명령**했다. 호출측이 이걸 보고 경고할 수 있게 노출한다.
    pub fn clamped_by_motor_limit(&self, joint_index: usize, angle_rad: f64) -> bool {
        let sign = f64::from(self.config.joint_signs[joint_index]);
        let adjusted = sign * angle_rad + self.config.joint_offsets_rad[joint_index];
        let raw = (f64::from(self.config.zero_tick)
            + adjusted * f64::from(self.config.ticks_per_revolution) / TAU)
            .round() as i32;
        let (lo, hi) = self.tick_limits[joint_index];
        return raw < lo || raw > hi;
    }

    pub fn ticks_to_radians(&self, joint_index: usize, ticks: i32) -> f64 {
        let raw = f64::from(ticks - self.config.zero_tick) * TAU
            / f64::from(self.config.ticks_per_revolution);
        let sign = f64::from(self.config.joint_signs[joint_index]);
        return sign * (raw - self.config.joint_offsets_rad[joint_index]);
    }
}
