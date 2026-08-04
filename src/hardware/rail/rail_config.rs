use std::path::PathBuf;

use super::rail_config_error::RailConfigError;
use super::soft_limit_args::SoftLimitArgs;

/// AXL 리니어 레일 설정.
///
/// 앱 벤치 값은 [`crate::defaults::hardware`]의 `Default`.
#[derive(Debug, Clone, PartialEq)]
pub struct RailConfig {
    pub enabled: bool,
    pub dll_path: PathBuf,
    pub axis: i32,
    pub irq_no: i32,
    pub pulses_per_meter: u32,
    /// `true`이면 앱 도메인과 AXL 보드의 좌표축 방향이 반대이다.
    /// 도메인 `[min, max]`는 보드 `[-max, -min]`에 대응한다.
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
        if !self.x_min_m.is_finite() || !self.x_max_m.is_finite() || self.x_min_m >= self.x_max_m {
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

    /// 절대 위치를 도메인 좌표에서 보드 좌표로 변환한다.
    /// `reverse`면 실제 AXL 엔코더 좌표처럼 부호를 반전한다.
    pub fn domain_to_board_abs(&self, domain_m: f64) -> f64 {
        if self.reverse {
            return -domain_m;
        }
        return domain_m;
    }

    /// 절대 위치: 보드(cmd/act) → 앱이 해석하는 도메인 좌표.
    pub fn board_to_domain_abs(&self, board_m: f64) -> f64 {
        if self.reverse {
            return -board_m;
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

    /// 도메인 이동 범위를 AXL 보드 좌표계의 양/음 소프트 리밋으로 변환한다.
    pub fn soft_limit_args(&self) -> SoftLimitArgs {
        let (positive_m, negative_m) = if self.reverse {
            (-self.x_min_m, -self.x_max_m)
        } else {
            (self.x_max_m, self.x_min_m)
        };
        return SoftLimitArgs {
            use_: 1,
            stop_mode: self.soft_limit_stop_mode,
            selection: self.soft_limit_selection,
            positive_m,
            negative_m,
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
            reverse: false,
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
    fn reverse_abs_and_soft_limits_use_negative_board_axis() {
        let cfg = RailConfig {
            reverse: true,
            x_min_m: 0.0,
            x_max_m: 1.43,
            ..RailConfig::default()
        };
        assert_eq!(cfg.domain_to_board_abs(0.0), 0.0);
        assert_eq!(cfg.domain_to_board_abs(1.43), -1.43);
        assert_eq!(cfg.domain_to_board_abs(0.2), -0.2);
        assert_eq!(cfg.board_to_domain_abs(-0.2), 0.2);
        assert_eq!(cfg.domain_to_board_rel(0.1), -0.1);
        assert_eq!(cfg.domain_to_board_rel(-0.05), 0.05);
        // 도메인 [0, 1.43]은 실제 AXL 보드 좌표 [-1.43, 0]이다.
        let args = cfg.soft_limit_args();
        assert_eq!(args.positive_m, 0.0);
        assert_eq!(args.negative_m, -1.43);
    }

    #[test]
    fn real_axl_negative_position_maps_to_expected_domain_position() {
        let cfg = RailConfig {
            reverse: true,
            x_min_m: 0.0,
            x_max_m: 1.41,
            ..RailConfig::default()
        };
        assert_eq!(cfg.board_to_domain_abs(-0.647304), 0.647304);
        assert_eq!(cfg.domain_to_board_abs(0.760), -0.760);
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
