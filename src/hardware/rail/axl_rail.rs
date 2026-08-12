//! AXL 리니어 레일 dry-run 및 Windows 실물 어댑터.

use crate::error::HwError;
use tracing::info;

use super::rail_config::{RailConfig, RailEnd};
use super::rail_kind::RailKind;

/// 팔 궤적 종료시간에 맞춰 계산한 레일 속도에 적용하는 배율.
/// 도착 시간에서 역산한 속도를 그대로 쓰고 추가 오버드라이브하지 않는다.
const COMMAND_SPEED_SCALE: f64 = 1.0;

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
            board_zero_domain_m = config.board_zero_domain_m,
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

    /// 저속으로 물리적 엔드스톱까지 이동해 AXL 알람으로 도달을 감지하고, 그 지점을
    /// 기준으로 `board_zero_domain_m`을 다시 계산한다. `DryRun`엔 물리 엔드스톱이 없어
    /// 항상 에러를 반환한다. 온디맨드 호출 전용 — 기동 시 자동으로 부르지 않는다.
    pub fn home(
        &mut self,
        #[cfg_attr(not(all(windows, feature = "real")), allow(unused_variables))] end: RailEnd,
    ) -> Result<RailHomeResult, HwError> {
        #[cfg(all(windows, feature = "real"))]
        if let RailKind::Live(live) = &mut self.kind {
            return home_live(live, &mut self.config, end);
        }
        return Err(HwError::InvalidConfig {
            reason: "AxlRail::home은 Live(실기) 레일에서만 지원됩니다".into(),
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
        let calculated_vel = velocity_for_distance_duration(distance_m, usable_duration, accel);
        let vel =
            (calculated_vel * COMMAND_SPEED_SCALE).clamp(self.config.min_vel, self.config.max_vel);
        let board_target_m = normalize_m(self.config.domain_to_board_abs(domain_m));
        info!(
            current_m,
            target_m = domain_m,
            board_target_m,
            calculated_velocity_m_s = calculated_vel,
            command_speed_scale = COMMAND_SPEED_SCALE,
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

    /// AXL 서보 알람 비트를 읽기만 한다 — 이동하지 않는다. 진단 전용:
    /// 정상 이동 명령이 에러 없이 반환되는데도 레일이 실제로 움직이지 않을 때,
    /// 원인이 AXL 쪽에 남아있는 래치된 알람인지 확인하는 데 쓴다. `DryRun`에는
    /// 알람 개념이 없어 에러를 반환한다.
    pub fn alarm_status(&mut self) -> Result<bool, HwError> {
        match &mut self.kind {
            RailKind::DryRun { .. } => Err(HwError::InvalidConfig {
                reason: "AxlRail::alarm_status는 Live(실기) 레일에서만 지원됩니다".into(),
            }),
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.read_alarm(self.config.axis),
        }
    }

    /// AXL 서보 알람을 해제한다(LOW→대기→HIGH→해제 대기→LOW 시퀀스) — 이동하지
    /// 않는다. `DryRun`에는 알람 개념이 없어 에러를 반환한다.
    pub fn clear_alarm(&mut self) -> Result<(), HwError> {
        match &mut self.kind {
            RailKind::DryRun { .. } => Err(HwError::InvalidConfig {
                reason: "AxlRail::clear_alarm은 Live(실기) 레일에서만 지원됩니다".into(),
            }),
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.reset_alarm(self.config.axis),
        }
    }
}

/// [`AxlRail::home`] 결과 — 캘리브레이션 파일에 그대로 옮겨 담는다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailHomeResult {
    /// 엔드스톱 도달 순간 읽은 원시 보드 좌표 [m] (`reverse` 해석 전).
    pub board_position_m: f64,
    /// 그 지점으로부터 역산한 새 영점.
    pub board_zero_domain_m: f64,
}

#[cfg(all(windows, feature = "real"))]
fn home_live(
    live: &mut super::axl_live::AxlLive,
    config: &mut RailConfig,
    end: RailEnd,
) -> Result<RailHomeResult, HwError> {
    use super::soft_limit_args::SoftLimitArgs;

    // 지금 설정된 소프트 리밋은 (틀렸을 수 있는) 기존 board_zero_domain_m으로
    // 계산돼 있다 — 실제 엔드스톱보다 먼저 이동을 멈출 수 있으므로 홈잉 중엔
    // 비활성화한다. 이동 결과와 무관하게 아래에서 항상 복원한다.
    let disabled_soft_limit = SoftLimitArgs {
        use_: 0,
        stop_mode: config.soft_limit_stop_mode,
        selection: config.soft_limit_selection,
        positive_m: 0.0,
        negative_m: 0.0,
    };
    live.set_soft_limit(config.axis, disabled_soft_limit)?;

    // 목표를 domain_to_board_abs(physical_x_{min,max}_m)으로 잡으면 그 변환 자체가
    // 재정렬하려는 board_zero_domain_m에 의존하는 순환 오류가 된다. 대신 **현재 보드
    // 위치 + 방향 × (전체 범위 + 여유)**로 좌표계 원점과 무관하게 목표를 잡는다.
    let move_result = home_move_toward_endstop(live, config, end);

    let board_position_m = match move_result {
        Ok(board_position_m) => board_position_m,
        Err(error) => {
            // 소프트 리밋을 비활성 상태로 남기지 않는다 — 실패해도 원래 설정으로 되돌린다.
            // 이 복구 자체가 실패하면 소프트 리밋이 꺼진 채로 남는 fail-open 상태이므로,
            // 조용히 삼키지 않고 반드시 로그로 남긴다 — 다음 이동부터 안전장치 없이
            // 움직인다는 것을 운용자가 알아야 한다.
            if let Err(restore_error) = live.set_soft_limit(config.axis, config.soft_limit_args())
            {
                tracing::error!(
                    axis = config.axis,
                    %restore_error,
                    "레일 홈잉 실패 후 소프트 리밋 복구도 실패 — 소프트 리밋이 비활성 상태로 남았습니다. 다음 이동 전 수동 확인 필요"
                );
            }
            return Err(error);
        }
    };

    let new_board_zero_domain_m = config.board_zero_domain_m_from_reference(end, board_position_m);
    config.board_zero_domain_m = new_board_zero_domain_m;
    live.set_soft_limit(config.axis, config.soft_limit_args())?;
    info!(
        axis = config.axis,
        end = ?end,
        board_position_m,
        new_board_zero_domain_m,
        "레일 홈잉 완료"
    );

    // 홈잉 직후 레일은 물리적 엔드스톱 근처 — 설정된 안전 이동 범위(x_min_m..x_max_m)
    // 밖일 수 있다. 그대로 두면 다음 실기 기동의 ready-pose 이동 계획이 "현재 위치가
    // 이미 범위 밖"이라는 이유로 가속도 한계를 넘어 실패한다. 안전 범위 안의 준비
    // 위치로 복귀시켜 다음 기동이 정상 범위에서 시작하게 한다.
    //
    // `move_abs_m_blocking`은 항상 `config.vel`(정상 운전 최고 속도, 기본
    // `RAIL_MAX_SPEED` 7.5 m/s)로 이동한다 — 엔드스톱에 막 부딪힌 직후 전속력 복귀는
    // 적절하지 않아 `start_move_abs_m`으로 감속된 `RAIL_HOMING_RETURN_VELOCITY_M_S`를
    // 직접 지정한다.
    let return_domain_m = config.clamp_m(crate::defaults::rail::RAIL_READY_X_M);
    let return_board_m = normalize_m(config.domain_to_board_abs(return_domain_m));
    live.start_move_abs_m(
        config,
        return_board_m,
        crate::defaults::rail::RAIL_HOMING_RETURN_VELOCITY_M_S,
    )?;
    live.wait_idle(config.axis)?;
    info!(
        axis = config.axis,
        return_domain_m, "레일 홈잉 후 준비 위치로 복귀"
    );

    return Ok(RailHomeResult {
        board_position_m,
        board_zero_domain_m: new_board_zero_domain_m,
    });
}

#[cfg(all(windows, feature = "real"))]
fn read_current_board_m(
    live: &mut super::axl_live::AxlLive,
    config: &RailConfig,
) -> Result<f64, HwError> {
    let (actual_board_m, _command_board_m) = live.read_actual_and_command_m(config.axis)?;
    return Ok(actual_board_m);
}

/// 현재 보드 위치 기준으로 엔드스톱 방향 목표를 잡고, 알람이 뜨거나 타임아웃할
/// 때까지 기다린다. 도달 시 정지·알람 해제까지 끝내고 그 순간의 원시 보드 좌표를
/// 반환한다.
#[cfg(all(windows, feature = "real"))]
fn home_move_toward_endstop(
    live: &mut super::axl_live::AxlLive,
    config: &RailConfig,
    end: RailEnd,
) -> Result<f64, HwError> {
    let current_board_m = read_current_board_m(live, config)?;
    let domain_direction = match end {
        RailEnd::Min => -1.0,
        RailEnd::Max => 1.0,
    };
    let board_direction = if config.reverse {
        -domain_direction
    } else {
        domain_direction
    };
    let travel_margin_m = (config.physical_x_max_m - config.physical_x_min_m).abs()
        + crate::defaults::rail::RAIL_HOMING_OVERTRAVEL_MARGIN_M;
    let target_board_m = normalize_m(current_board_m + board_direction * travel_margin_m);

    live.start_move_abs_m(
        config,
        target_board_m,
        crate::defaults::rail::RAIL_HOMING_VELOCITY_M_S,
    )?;

    let deadline = std::time::Instant::now()
        + std::time::Duration::from_secs_f64(crate::defaults::rail::RAIL_HOMING_TIMEOUT_SECS);
    let mut last_progress_log = std::time::Instant::now();
    loop {
        if live.read_alarm(config.axis)? {
            break;
        }
        if std::time::Instant::now() >= deadline {
            live.stop(config.axis)?;
            return Err(HwError::InvalidConfig {
                reason: "레일 홈잉: 엔드스톱 도달 못 함 — 배선/알람 설정 확인".into(),
            });
        }
        if last_progress_log.elapsed() >= std::time::Duration::from_secs(2) {
            if let Ok(actual_board_m) = read_current_board_m(live, config) {
                info!(
                    axis = config.axis,
                    actual_board_m, target_board_m, "레일 홈잉 진행 중 — 아직 엔드스톱 미도달"
                );
            }
            last_progress_log = std::time::Instant::now();
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }

    live.stop(config.axis)?;
    let board_position_m = read_current_board_m(live, config)?;
    live.reset_alarm(config.axis)?;
    return Ok(board_position_m);
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
    fn command_abs_in_secs_always_keeps_safety_margin() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            x_min_m: 0.01,
            x_max_m: 1.3395,
            physical_x_min_m: 0.0,
            physical_x_max_m: 1.41,
            ..RailConfig::default()
        };
        let mut rail = AxlRail::dry_run(cfg).unwrap();
        assert_eq!(rail.command_abs_in_secs(1.40, 0.4).unwrap(), 1.3395);
        assert_eq!(rail.read_x_m().unwrap(), 1.3395);
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
            board_zero_domain_m: 0.2,
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
        // reverse=true이면 AXL 보드 0이 명시한 도메인 원점 0.2에 대응한다.
        assert_eq!(rail.read_board_x_m().unwrap(), -0.05);
        let commanded = rail.move_rel_m(0.05).unwrap();
        assert_eq!(commanded, 0.3);
        assert_eq!(rail.read_board_x_m().unwrap(), -0.1);
    }

    #[test]
    fn home_rejects_dry_run() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            x_min_m: 0.0,
            x_max_m: 1.0,
            ..RailConfig::default()
        };
        let mut rail = AxlRail::dry_run(cfg).unwrap();
        assert!(rail.home(crate::hardware::rail::RailEnd::Min).is_err());
    }

    #[test]
    fn alarm_status_and_clear_alarm_reject_dry_run() {
        let cfg = RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            x_min_m: 0.0,
            x_max_m: 1.0,
            ..RailConfig::default()
        };
        let mut rail = AxlRail::dry_run(cfg).unwrap();
        assert!(rail.alarm_status().is_err());
        assert!(rail.clear_alarm().is_err());
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
