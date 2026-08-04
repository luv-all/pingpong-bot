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
        let (board_position_m, board_command_position_m) =
            live.read_actual_and_command_m(config.axis)?;
        let domain_position_m = config.board_to_domain_abs(board_position_m);
        let board_limits = config.soft_limit_args();
        info!(
            axis = config.axis,
            board_position_m,
            board_command_position_m,
            board_command_minus_actual_m = board_command_position_m - board_position_m,
            domain_position_m,
            configured_domain_min_m = config.x_min_m,
            configured_domain_max_m = config.x_max_m,
            configured_board_min_m = board_limits.negative_m,
            configured_board_max_m = board_limits.positive_m,
            reverse = config.reverse,
            pulses_per_meter = config.pulses_per_meter,
            "AXL 시작 좌표 진단"
        );

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

    /// 가속·감속 램프를 포함해 `duration_secs`에 도착할 속도를 계산한다.
    pub fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError> {
        let prepare_started = std::time::Instant::now();
        #[cfg(all(windows, feature = "real"))]
        if let RailKind::Live(live) = &mut self.kind {
            // 이전 1차 이동을 먼저 정지한 후의 실제 위치로 속도를 다시 계산한다.
            live.stop_if_moving(self.config.axis)?;
        }
        let usable_duration =
            (duration_secs - prepare_started.elapsed().as_secs_f64()).max(f64::EPSILON);
        let domain_m = normalize_m(self.config.clamp_m(x));
        let current_m = self.read_x_m()?;
        let distance_m = (domain_m - current_m).abs();
        if distance_m <= 1e-9 || usable_duration <= f64::EPSILON {
            return self.set_domain_position(domain_m);
        }

        let accel = self.config.accel.min(self.config.decel);
        let vel = velocity_for_distance_duration(distance_m, usable_duration, accel)
            .clamp(self.config.min_vel, self.config.max_vel);
        let board_target_m = normalize_m(self.config.domain_to_board_abs(domain_m));
        info!(
            current_m,
            target_m = domain_m,
            board_target_m,
            velocity_m_s = vel,
            duration_secs,
            usable_duration_secs = usable_duration,
            "AXL 레일 이동 명령"
        );
        match &mut self.kind {
            RailKind::DryRun { position_m } => {
                let _ = vel;
                *position_m = domain_m;
            }
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => {
                live.start_move_abs_m(&self.config, board_target_m, vel)?;
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

    /// 진행 중인 레일 이동을 부드럽게 정지시킨다.
    pub fn stop(&mut self) -> Result<(), HwError> {
        match &mut self.kind {
            RailKind::DryRun { .. } => Ok(()),
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.stop(self.config.axis),
        }
    }
}

fn velocity_for_distance_duration(distance: f64, duration: f64, acceleration: f64) -> f64 {
    let discriminant = duration * duration - 4.0 * distance / acceleration;
    if discriminant <= 0.0 {
        return f64::INFINITY;
    }
    return 0.5 * acceleration * (duration - discriminant.sqrt());
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

    use super::{AxlRail, velocity_for_distance_duration};
    use crate::hardware::rail::RailConfig;

    #[test]
    fn command_abs_in_secs_reaches_requested_target() {
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
    fn duration_velocity_includes_acceleration_and_deceleration() {
        let distance = 0.2;
        let duration = 0.4;
        let acceleration = 12.0;
        let velocity = velocity_for_distance_duration(distance, duration, acceleration);
        let resulting_duration = distance / velocity + velocity / acceleration;
        assert!((resulting_duration - duration).abs() < 1e-12);
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
        // reverse=true이면 AXL 보드 중앙 0이 도메인 중점 0.2에 대응한다.
        assert_eq!(rail.read_board_x_m().unwrap(), -0.05);
        let commanded = rail.move_rel_m(0.05).unwrap();
        assert_eq!(commanded, 0.3);
        assert_eq!(rail.read_board_x_m().unwrap(), -0.1);
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
