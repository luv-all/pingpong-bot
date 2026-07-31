//! AXL 리니어 레일 dry-run 및 Windows 실물 어댑터.

use crate::error::HwError;
use tracing::info;

use super::rail_config::RailConfig;
use super::rail_kind::RailKind;

pub struct AxlRail {
    config: RailConfig,
    kind: RailKind,
}

impl AxlRail {
    /// DLL 없이 레일 좌표·클램프 경로를 검증한다.
    pub fn dry_run(config: RailConfig) -> Result<Self, HwError> {
        validate_config(&config)?;
        return Ok(Self {
            config,
            kind: RailKind::DryRun { position_m: 0.0 },
        });
    }

    /// Windows AXL DLL을 열고 단일 축을 초기화한다.
    #[cfg(all(windows, feature = "real"))]
    pub fn open(config: RailConfig) -> Result<Self, HwError> {
        validate_config(&config)?;
        if !config.enabled {
            return Err(HwError::InvalidConfig {
                reason: "enabled=true인 rail 설정이 필요합니다".into(),
            });
        }

        let ffi = super::axl_ffi::AxlFfi::load(&config.dll_path)?;
        tracing::debug!(
            dll = %config.dll_path.display(),
            axis = config.axis,
            irq_no = config.irq_no,
            "AXL DLL 로드 · AxlOpenNoReset"
        );
        // 칩 리셋 없이 열어 보드에 기록된 엔코더/명령 위치를 유지한다.
        super::axl_live::check_axl("AxlOpenNoReset", unsafe {
            (ffi.axl_open_no_reset)(config.irq_no)
        })?;
        let mut live = super::axl_live::AxlLive::new(ffi);
        live.configure(&config)?;
        tracing::debug!(axis = config.axis, "AXL 축 설정·서보 ON 완료");

        return Ok(Self {
            config,
            kind: RailKind::Live(live),
        });
    }

    /// Windows+real이 아닌 빌드에서는 실물 AXL 장치를 열 수 없다.
    #[cfg(not(all(windows, feature = "real")))]
    pub fn open(_config: RailConfig) -> Result<Self, HwError> {
        return Err(HwError::InvalidConfig {
            reason: "AxlRail::open은 Windows + feature=real 에서만 지원됩니다".into(),
        });
    }

    /// 보드 cmd/act 원시 위치 [m]. `reverse` 해석 전 값.
    pub fn read_board_x_m(&mut self) -> Result<f64, HwError> {
        match &mut self.kind {
            RailKind::DryRun { position_m } => {
                Ok(normalize_m(self.config.domain_to_board_abs(*position_m)))
            }
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.read_x_m(self.config.axis),
        }
    }

    /// 앱이 해석하는 도메인 위치 [m] (`reverse` 반영).
    pub fn read_x_m(&mut self) -> Result<f64, HwError> {
        let board_m = self.read_board_x_m()?;
        return Ok(normalize_m(self.config.board_to_domain_abs(board_m)));
    }

    /// 궤적 시작 시 한 번만 레일을 이동한다. 속도는 `|Δx|/duration` (램프 무시).
    pub fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError> {
        let domain_m = normalize_m(self.config.clamp_m(x));
        let current_m = self.read_x_m()?;
        let distance_m = (domain_m - current_m).abs();
        if distance_m <= 1e-9 || duration_secs <= f64::EPSILON {
            return self.set_domain_position(domain_m);
        }

        let vel = (distance_m / duration_secs).clamp(self.config.min_vel, self.config.max_vel);
        info!(
            current_m,
            target_m = domain_m,
            velocity_m_s = vel,
            duration_secs,
            "AXL 레일 이동 명령"
        );
        match &mut self.kind {
            RailKind::DryRun { position_m } => {
                let _ = vel;
                *position_m = domain_m;
            }
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => {
                let board_m = normalize_m(self.config.domain_to_board_abs(domain_m));
                live.start_move_abs_m(&self.config, board_m, vel)?;
            }
        }
        return Ok(domain_m);
    }

    fn set_domain_position(&mut self, domain_m: f64) -> Result<f64, HwError> {
        match &mut self.kind {
            RailKind::DryRun { position_m } => *position_m = domain_m,
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(_) => {}
        }
        return Ok(domain_m);
    }

    /// 도메인 절대 목표를 건다. Live는 `AxmMovePos`로 블로킹 이동한다.
    pub fn command_abs_m(&mut self, x: f64) -> Result<f64, HwError> {
        let domain_m = normalize_m(self.config.clamp_m(x));
        match &mut self.kind {
            RailKind::DryRun { position_m } => *position_m = domain_m,
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => {
                let board_m = normalize_m(self.config.domain_to_board_abs(domain_m));
                live.move_abs_m_blocking(&self.config, board_m)?;
            }
        }
        return Ok(domain_m);
    }

    /// 도메인 절대 위치로 이동하고(클램프), Live면 정지까지 기다린다. 반환값은 도메인 명령.
    pub fn move_abs_m(&mut self, x: f64) -> Result<f64, HwError> {
        let domain_m = self.command_abs_m(x)?;
        #[cfg(all(windows, feature = "real"))]
        if let RailKind::Live(live) = &mut self.kind {
            live.wait_idle(self.config.axis)?;
        }
        return Ok(domain_m);
    }

    /// 도메인 상대 이동. `reverse`면 보드 Δ에 -1을 곱한 것과 같다.
    pub fn move_rel_m(&mut self, dx: f64) -> Result<f64, HwError> {
        let current_domain = self.read_x_m()?;
        return self.move_abs_m(current_domain + dx);
    }

    /// 진행 중 절대 이동이 실제로 끝날 때까지 기다린다.
    pub fn wait_idle(&mut self) -> Result<(), HwError> {
        match &mut self.kind {
            RailKind::DryRun { .. } => Ok(()),
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.wait_idle(self.config.axis),
        }
    }
}

fn normalize_m(x: f64) -> f64 {
    return (x * 1_000_000_000_000.0).round() / 1_000_000_000_000.0;
}

fn validate_config(config: &RailConfig) -> Result<(), HwError> {
    return config.validate().map_err(|error| HwError::InvalidConfig {
        reason: error.to_string(),
    });
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::AxlRail;
    use crate::hardware::rail::RailConfig;

    #[test]
    fn command_abs_in_secs_uses_distance_over_duration() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            x_min_m: 0.0,
            x_max_m: 1.0,
            min_vel: 0.01,
            max_vel: 2.0,
            ..RailConfig::default()
        };
        let mut rail = AxlRail::dry_run(cfg).unwrap();
        let commanded = rail.command_abs_in_secs(0.2, 0.4).unwrap();
        assert_eq!(commanded, 0.2);
        assert_eq!(rail.read_x_m().unwrap(), 0.2);
    }

    #[test]
    fn command_abs_in_secs_skips_zero_move() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            x_min_m: 0.0,
            x_max_m: 1.0,
            ..RailConfig::default()
        };
        let mut rail = AxlRail::dry_run(cfg).unwrap();
        rail.command_abs_in_secs(0.0, 0.5).unwrap();
        let commanded = rail.command_abs_in_secs(0.0, 0.5).unwrap();
        assert_eq!(commanded, 0.0);
    }

    #[test]
    fn dry_run_move_abs_clamps_and_updates_position() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            x_min_m: 0.0,
            x_max_m: 0.4,
            vel: 0.2,
            accel: 1.0,
            decel: 1.0,
            min_vel: 0.001,
            max_vel: 1.0,
            ..RailConfig::default()
        };
        let mut rail = AxlRail::dry_run(cfg).unwrap();
        assert_eq!(rail.read_x_m().unwrap(), 0.0);
        let commanded = rail.move_abs_m(1.0).unwrap();
        assert_eq!(commanded, 0.4);
        assert_eq!(rail.read_x_m().unwrap(), 0.4);
        let commanded = rail.move_rel_m(-0.1).unwrap();
        assert_eq!(commanded, 0.3);
    }

    #[test]
    fn dry_run_reverse_maps_abs_min_max_and_keeps_domain_api() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            reverse: true,
            x_min_m: 0.0,
            x_max_m: 0.4,
            vel: 0.2,
            accel: 1.0,
            decel: 1.0,
            min_vel: 0.001,
            max_vel: 1.0,
            ..RailConfig::default()
        };
        let mut rail = AxlRail::dry_run(cfg).unwrap();
        let commanded = rail.move_abs_m(0.25).unwrap();
        assert_eq!(commanded, 0.25);
        assert_eq!(rail.read_x_m().unwrap(), 0.25);
        // board abs = xmin + xmax - domain
        assert_eq!(rail.read_board_x_m().unwrap(), 0.15);
        let commanded = rail.move_rel_m(0.05).unwrap();
        assert_eq!(commanded, 0.3);
        assert_eq!(rail.read_board_x_m().unwrap(), 0.1);
    }

    #[cfg(all(windows, feature = "real"))]
    #[test]
    fn read_error_includes_both_axl_status_codes() {
        let error = super::super::axl_live::read_position_error(7, 9);

        assert_eq!(
            error,
            crate::error::HwError::InvalidConfig {
                reason: "AXL AxmStatusGetActPos code=7; AxmStatusGetCmdPos code=9".into(),
            }
        );
    }

    #[cfg(all(windows, feature = "real"))]
    #[test]
    fn move_poll_timeout_error_identifies_axl_operation() {
        let error = super::super::axl_live::move_poll_timeout_error();

        assert_eq!(
            error,
            crate::error::HwError::InvalidConfig {
                reason: "AXL AxmStatusReadInMotion timeout after 30s".into(),
            }
        );
    }
}
