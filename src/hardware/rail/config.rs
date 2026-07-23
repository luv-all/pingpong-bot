//! AXL 리니어 레일 설정·클램프·soft-limit 인자.

use std::path::PathBuf;

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

/// AXL 리니어 레일 설정.
///
/// 앱 벤치 값은 [`crate::defaults::rail`]. 여기 `Default`는 비활성 골격이다.
#[derive(Debug, Clone, PartialEq)]
pub struct RailConfig {
    pub enabled: bool,
    pub dll_path: PathBuf,
    pub axis: i32,
    pub irq_no: i32,
    pub pulses_per_meter: u32,
    /// `true`이면 앱 도메인과 보드 cmd/act 절대 좌표를 min↔max로 대응시킨다.
    /// 상대 이동량만 부호 반전한다.
    pub reverse: bool,
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
            reverse: false,
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

/// 레일 설정 검증 실패.
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

    /// 절대 위치: 도메인 → 보드.
    /// `reverse`면 구간 끝점을 서로 대응시킨다 (`domain_min ↔ board_max`).
    pub fn domain_to_board_abs(&self, domain_m: f64) -> f64 {
        if self.reverse {
            return self.x_min_m + self.x_max_m - domain_m;
        }
        return domain_m;
    }

    /// 절대 위치: 보드(cmd/act) → 앱이 해석하는 도메인 좌표.
    pub fn board_to_domain_abs(&self, board_m: f64) -> f64 {
        if self.reverse {
            return self.x_min_m + self.x_max_m - board_m;
        }
        return board_m;
    }

    /// 상대 이동량: 도메인 Δ → 보드 Δ. `reverse`면 부호만 반전.
    pub fn domain_to_board_rel(&self, domain_dx: f64) -> f64 {
        if self.reverse {
            return -domain_dx;
        }
        return domain_dx;
    }

    /// Soft limit는 보드 물리 좌표의 이동 한도(도메인 해석과 무관).
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
    fn soft_limit_args_mirror_meters() {
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
    fn reverse_abs_reflects_min_max_rel_negates() {
        let cfg = RailConfig {
            reverse: true,
            x_min_m: 0.0,
            x_max_m: 1.43,
            ..RailConfig::default()
        };
        assert_eq!(cfg.domain_to_board_abs(0.0), 1.43);
        assert_eq!(cfg.domain_to_board_abs(1.43), 0.0);
        assert!((cfg.domain_to_board_abs(0.2) - 1.23).abs() < 1e-12);
        assert!((cfg.board_to_domain_abs(1.23) - 0.2).abs() < 1e-12);
        assert_eq!(cfg.domain_to_board_rel(0.1), -0.1);
        assert_eq!(cfg.domain_to_board_rel(-0.05), 0.05);
        // soft limit는 보드 물리 한도 그대로
        let args = cfg.soft_limit_args();
        assert_eq!(args.positive_m, 1.43);
        assert_eq!(args.negative_m, 0.0);
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
