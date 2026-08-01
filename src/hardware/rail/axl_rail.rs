//! AXL 리니어 레일 dry-run 및 Windows 실물 어댑터.

use std::time::Instant;

use crate::error::HwError;
use tracing::info;

use super::rail_config::RailConfig;
use super::rail_kind::RailKind;

/// 짧은 거리도 느린 등속 명령으로 늘어지지 않게 한다.
/// 실제 도달 속도는 AXL의 가감속 프로파일과 이동 거리가 제한한다.
const DIRECT_MIN_PEAK_VEL_M_S: f64 = 2.0;

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

    /// 궤적 시작 시 한 번만 레일을 이동한다.
    /// 재목표 감속 정지 후 실제 위치·남은 시간과 가감속 램프를 반영한다.
    pub fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError> {
        let started_at = Instant::now();
        let domain_m = normalize_m(self.config.clamp_m(x));
        let retargeted = match &mut self.kind {
            RailKind::DryRun { .. } => false,
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.stop_for_retarget(self.config.axis)?,
        };
        let current_m = self.read_x_m()?;
        let distance_m = (domain_m - current_m).abs();
        if distance_m <= 1e-9 || duration_secs <= f64::EPSILON {
            return self.set_domain_position(domain_m);
        }

        let stop_elapsed_secs = started_at.elapsed().as_secs_f64();
        let usable_duration_secs = (duration_secs - stop_elapsed_secs).max(f64::EPSILON);
        let direct_min_vel = self
            .config
            .min_vel
            .max(DIRECT_MIN_PEAK_VEL_M_S)
            .min(self.config.max_vel);
        let vel = ramp_compensated_velocity(
            distance_m,
            usable_duration_secs,
            self.config.accel,
            self.config.decel,
            direct_min_vel,
            self.config.max_vel,
        );
        let board_current_m = normalize_m(self.config.domain_to_board_abs(current_m));
        let board_target_m = normalize_m(self.config.domain_to_board_abs(domain_m));
        let board_delta_m = board_target_m - board_current_m;
        // AXL 보드 +는 발사기에서 볼 때 오른쪽이다.
        let launcher_view_direction = if board_delta_m > 1e-6 {
            "오른쪽"
        } else if board_delta_m < -1e-6 {
            "왼쪽"
        } else {
            "정지"
        };
        info!(
            current_m,
            target_m = domain_m,
            board_current_m,
            board_target_m,
            board_delta_m,
            launcher_view_direction,
            reverse = self.config.reverse,
            retargeted,
            stop_elapsed_secs,
            velocity_m_s = vel,
            requested_duration_secs = duration_secs,
            usable_duration_secs,
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
}

/// 유한한 가속·감속으로 잃는 거리를 보상한 피크 속도.
/// 주어진 시간이 물리적으로 너무 짧으면 허용 최대 속도를 쓴다.
fn ramp_compensated_velocity(
    distance_m: f64,
    duration_secs: f64,
    accel_m_s2: f64,
    decel_m_s2: f64,
    min_vel_m_s: f64,
    max_vel_m_s: f64,
) -> f64 {
    let base = distance_m / duration_secs.max(f64::EPSILON);
    let ramp = 0.5 / accel_m_s2.max(f64::EPSILON) + 0.5 / decel_m_s2.max(f64::EPSILON);
    let discriminant = duration_secs * duration_secs - 4.0 * ramp * distance_m;
    let compensated = if discriminant >= 0.0 {
        (duration_secs - discriminant.sqrt()) / (2.0 * ramp)
    } else {
        max_vel_m_s
    };
    return compensated.max(base).clamp(min_vel_m_s, max_vel_m_s);
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

    use super::{AxlRail, ramp_compensated_velocity};
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
    fn short_move_uses_fast_minimum_peak_velocity() {
        let vel = ramp_compensated_velocity(0.05, 0.30, 12.0, 12.0, 2.0, 7.0);
        assert_eq!(vel, 2.0);
    }

    #[test]
    fn impossible_deadline_uses_allowed_maximum_velocity() {
        let vel = ramp_compensated_velocity(0.70, 0.12, 12.0, 12.0, 2.0, 7.0);
        assert_eq!(vel, 7.0);
    }

    #[test]
    fn ramp_compensation_is_faster_than_distance_over_time() {
        let distance = 0.15;
        let duration = 0.26;
        let vel = ramp_compensated_velocity(distance, duration, 12.0, 12.0, 0.001, 7.0);
        assert!(vel > distance / duration);
        assert!(vel <= 7.0);
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
