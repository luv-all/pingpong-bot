//! AXL 리니어 레일 TOML 설정·클램프·soft-limit 인자.

use std::path::{Path, PathBuf};

use serde::Deserialize;
use thiserror::Error;

/// `AxmSignalSetSoftLimit` 인자 (미터 단위).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SoftLimitArgs {
    pub use_: u32,
    pub stop_mode: u32,
    pub selection: u32,
    pub positive_m: f64,
    pub negative_m: f64,
}

/// AXL 리니어 레일 TOML 설정.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default)]
pub struct RailConfig {
    pub enabled: bool,
    pub dll_path: PathBuf,
    pub axis: i32,
    pub irq_no: i32,
    pub pulses_per_meter: u32,
    pub x_min_m: f64,
    pub x_max_m: f64,
    pub vel: f64,
    pub accel: f64,
    pub decel: f64,
    pub min_vel: f64,
    pub max_vel: f64,
    pub pulse_out_method: u32,
    pub enc_input_method: u32,
    pub abs_rel_mode: u32,
    pub profile_mode: u32,
    pub accel_unit: u32,
    pub soft_limit_stop_mode: u32,
    pub soft_limit_selection: u32,
    pub inposition_use: u32,
    pub alarm_use: u32,
    pub limit_stop_mode: u32,
    pub pos_end_limit_level: u32,
    pub neg_end_limit_level: u32,
}

impl Default for RailConfig {
    fn default() -> Self {
        return Self {
            enabled: false,
            dll_path: PathBuf::new(),
            axis: 0,
            irq_no: 7,
            pulses_per_meter: 2_500_000,
            x_min_m: -0.20,
            x_max_m: 0.50,
            vel: 0.3,
            accel: 1.0,
            decel: 1.0,
            min_vel: 0.001,
            max_vel: 1.0,
            pulse_out_method: 4,
            enc_input_method: 3,
            abs_rel_mode: 0,
            profile_mode: 3,
            accel_unit: 0,
            soft_limit_stop_mode: 0,
            soft_limit_selection: 0,
            inposition_use: 1,
            alarm_use: 0,
            limit_stop_mode: 0,
            pos_end_limit_level: 2,
            neg_end_limit_level: 2,
        };
    }
}

/// 레일 TOML/설정 검증·로드 실패.
#[derive(Debug, Error)]
pub enum RailConfigError {
    #[error("enabled=true일 때 dll_path는 비어 있으면 안 됩니다")]
    DllPathEmpty,
    #[error("enabled=true일 때 pulses_per_meter는 0보다 커야 합니다")]
    PulsesPerMeter,
    #[error("x_min_m은 x_max_m보다 작아야 합니다")]
    InvalidRange,
    #[error("motion 파라미터가 유효하지 않습니다")]
    MotionParams,
    #[error("TOML 파싱 실패: {0}")]
    Toml(#[from] toml::de::Error),
    #[error("설정 파일 읽기 실패: {0}")]
    Io(#[from] std::io::Error),
}

impl RailConfig {
    pub fn validate(&self) -> Result<(), RailConfigError> {
        if !self.enabled {
            return Ok(());
        }
        if self.dll_path.as_os_str().is_empty() {
            return Err(RailConfigError::DllPathEmpty);
        }
        if self.pulses_per_meter == 0 {
            return Err(RailConfigError::PulsesPerMeter);
        }
        if !self.x_min_m.is_finite()
            || !self.x_max_m.is_finite()
            || self.x_min_m >= self.x_max_m
        {
            return Err(RailConfigError::InvalidRange);
        }
        for value in [self.vel, self.accel, self.decel, self.max_vel] {
            if !value.is_finite() || value <= 0.0 {
                return Err(RailConfigError::MotionParams);
            }
        }
        if !self.min_vel.is_finite() || self.min_vel <= 0.0 {
            return Err(RailConfigError::MotionParams);
        }
        return Ok(());
    }

    pub fn clamp_m(&self, x: f64) -> f64 {
        return x.clamp(self.x_min_m, self.x_max_m);
    }

    pub fn soft_limit_args(&self) -> SoftLimitArgs {
        return SoftLimitArgs {
            use_: 1,
            stop_mode: self.soft_limit_stop_mode,
            selection: self.soft_limit_selection,
            positive_m: self.x_max_m,
            negative_m: self.x_min_m,
        };
    }
}

#[derive(Deserialize)]
struct RuntimeHardwareDocument {
    hardware: RuntimeHardwareSection,
}

#[derive(Deserialize)]
struct RuntimeHardwareSection {
    rail: RailConfig,
}

/// 전체 런타임 TOML에서 `[hardware.rail]`만 읽는다.
pub fn config_from_toml(text: &str) -> Result<RailConfig, RailConfigError> {
    let document: RuntimeHardwareDocument = toml::from_str(text)?;
    document.hardware.rail.validate()?;
    return Ok(document.hardware.rail);
}

pub fn load_rail_config(path: &Path) -> Result<RailConfig, RailConfigError> {
    let text = std::fs::read_to_string(path)?;
    return config_from_toml(&text);
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::RailConfig;

    #[test]
    fn clamp_rail_m_respects_limits() {
        let cfg = RailConfig {
            x_min_m: -0.2,
            x_max_m: 0.5,
            ..RailConfig::default()
        };
        assert_eq!(cfg.clamp_m(-1.0), -0.2);
        assert_eq!(cfg.clamp_m(0.1), 0.1);
        assert_eq!(cfg.clamp_m(2.0), 0.5);
    }

    #[test]
    fn validate_rejects_bad_range_when_enabled() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("AXL.dll"),
            pulses_per_meter: 2500,
            x_min_m: 0.5,
            x_max_m: -0.5,
            ..RailConfig::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn soft_limit_args_mirror_toml_meters() {
        let cfg = RailConfig {
            x_min_m: -0.15,
            x_max_m: 0.40,
            soft_limit_stop_mode: 0,
            soft_limit_selection: 0,
            ..RailConfig::default()
        };
        let args = cfg.soft_limit_args();
        assert_eq!(args.use_, 1);
        assert_eq!(args.positive_m, 0.40);
        assert_eq!(args.negative_m, -0.15);
    }

    #[test]
    fn disabled_config_skips_path_checks() {
        let cfg = RailConfig {
            enabled: false,
            dll_path: PathBuf::new(),
            pulses_per_meter: 0,
            x_min_m: 0.0,
            x_max_m: 0.0,
            ..RailConfig::default()
        };
        assert!(cfg.validate().is_ok());
    }
}
