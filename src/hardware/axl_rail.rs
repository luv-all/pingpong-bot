//! AXL 리니어 레일 dry-run 및 Windows 실물 어댑터.

use crate::HwError;

use super::rail::RailConfig;

pub struct AxlRail {
    config: RailConfig,
    kind: RailKind,
}

enum RailKind {
    DryRun {
        position_m: f64,
    },
    #[cfg(all(windows, feature = "real"))]
    Live(AxlLive),
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
        check_axl("AxlOpen", unsafe { (ffi.axl_open)(config.irq_no) })?;
        let mut live = AxlLive {
            ffi,
            axis: config.axis,
        };
        live.configure(&config)?;

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

    pub fn read_x_m(&mut self) -> Result<f64, HwError> {
        match &mut self.kind {
            RailKind::DryRun { position_m } => Ok(*position_m),
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.read_x_m(self.config.axis),
        }
    }

    /// 절대 위치 명령을 레일 소프트 리밋으로 클램프하고, 실제 명령값을 반환한다.
    pub fn move_abs_m(&mut self, x: f64) -> Result<f64, HwError> {
        let commanded_m = normalize_m(self.config.clamp_m(x));
        match &mut self.kind {
            RailKind::DryRun { position_m } => *position_m = commanded_m,
            #[cfg(all(windows, feature = "real"))]
            RailKind::Live(live) => live.move_abs_m(&self.config, commanded_m)?,
        }
        return Ok(commanded_m);
    }

    pub fn move_rel_m(&mut self, dx: f64) -> Result<f64, HwError> {
        let current_m = self.read_x_m()?;
        return self.move_abs_m(current_m + dx);
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

#[cfg(all(windows, feature = "real"))]
struct AxlLive {
    ffi: super::axl_ffi::AxlFfi,
    axis: i32,
}

#[cfg(all(windows, feature = "real"))]
impl AxlLive {
    fn configure(&mut self, config: &RailConfig) -> Result<(), HwError> {
        let axis = config.axis;
        let mut status = 0;
        check_axl("AxmInfoIsMotionModule", unsafe {
            (self.ffi.axm_info_is_motion_module)(&mut status)
        })?;
        if status != super::axl_ffi::STATUS_EXIST {
            return Err(HwError::InvalidConfig {
                reason: format!("AXL motion module axis={axis} status={status}"),
            });
        }

        check_axl("AxmMotSetPulseOutMethod", unsafe {
            (self.ffi.axm_mot_set_pulse_out_method)(axis, config.pulse_out_method)
        })?;
        check_axl("AxmMotSetEncInputMethod", unsafe {
            (self.ffi.axm_mot_set_enc_input_method)(axis, config.enc_input_method)
        })?;
        check_axl("AxmMotSetMoveUnitPerPulse", unsafe {
            (self.ffi.axm_mot_set_move_unit_per_pulse)(
                axis,
                1.0 / f64::from(config.pulses_per_meter),
                1,
            )
        })?;
        check_axl("AxmMotSetMinVel", unsafe {
            (self.ffi.axm_mot_set_min_vel)(axis, config.min_vel)
        })?;
        check_axl("AxmMotSetMaxVel", unsafe {
            (self.ffi.axm_mot_set_max_vel)(axis, config.max_vel)
        })?;
        check_axl("AxmMotSetAccelUnit", unsafe {
            (self.ffi.axm_mot_set_accel_unit)(axis, config.accel_unit)
        })?;
        check_axl("AxmMotSetAbsRelMode", unsafe {
            (self.ffi.axm_mot_set_abs_rel_mode)(axis, config.abs_rel_mode)
        })?;
        check_axl("AxmMotSetProfileMode", unsafe {
            (self.ffi.axm_mot_set_profile_mode)(axis, config.profile_mode)
        })?;
        check_axl("AxmSignalSetInpos", unsafe {
            (self.ffi.axm_signal_set_inpos)(axis, config.inposition_use)
        })?;
        check_axl("AxmSignalSetServoAlarm", unsafe {
            (self.ffi.axm_signal_set_servo_alarm)(axis, config.alarm_use)
        })?;
        check_axl("AxmSignalSetLimit", unsafe {
            (self.ffi.axm_signal_set_limit)(
                axis,
                config.limit_stop_mode,
                config.pos_end_limit_level,
                config.neg_end_limit_level,
            )
        })?;
        let soft_limit = config.soft_limit_args();
        check_axl("AxmSignalSetSoftLimit", unsafe {
            (self.ffi.axm_signal_set_soft_limit)(
                axis,
                soft_limit.use_,
                soft_limit.stop_mode,
                soft_limit.selection,
                soft_limit.positive_m,
                soft_limit.negative_m,
            )
        })?;
        return check_axl("AxmSignalServoOn", unsafe {
            (self.ffi.axm_signal_servo_on)(axis, super::axl_ffi::ENABLE)
        });
    }

    fn read_x_m(&mut self, axis: i32) -> Result<f64, HwError> {
        let mut position_m = 0.0;
        let actual_status = unsafe { (self.ffi.axm_status_get_act_pos)(axis, &mut position_m) };
        if actual_status == super::axl_ffi::AXT_RT_SUCCESS {
            return Ok(position_m);
        }

        let command_status = unsafe { (self.ffi.axm_status_get_cmd_pos)(axis, &mut position_m) };
        if command_status == super::axl_ffi::AXT_RT_SUCCESS {
            return Ok(position_m);
        }
        return Err(HwError::ReadFailed);
    }

    fn move_abs_m(&mut self, config: &RailConfig, commanded_m: f64) -> Result<(), HwError> {
        check_axl("AxmMotSetAbsRelMode", unsafe {
            (self.ffi.axm_mot_set_abs_rel_mode)(config.axis, 0)
        })?;
        check_axl("AxmMovePos", unsafe {
            (self.ffi.axm_move_pos)(
                config.axis,
                commanded_m,
                config.vel,
                config.accel,
                config.decel,
            )
        })?;

        loop {
            let mut in_motion = 0;
            check_axl("AxmStatusReadInMotion", unsafe {
                (self.ffi.axm_status_read_in_motion)(config.axis, &mut in_motion)
            })?;
            if in_motion == 0 {
                return Ok(());
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}

#[cfg(all(windows, feature = "real"))]
impl Drop for AxlLive {
    fn drop(&mut self) {
        let _ = unsafe { (self.ffi.axm_signal_servo_on)(self.axis, 0) };
        let _ = unsafe { (self.ffi.axl_close)() };
    }
}

#[cfg(all(windows, feature = "real"))]
fn check_axl(name: &str, code: u32) -> Result<(), HwError> {
    if code == super::axl_ffi::AXT_RT_SUCCESS {
        return Ok(());
    }
    return Err(HwError::InvalidConfig {
        reason: format!("AXL {name} code={code}"),
    });
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::AxlRail;
    use crate::hardware::rail::RailConfig;

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
}
