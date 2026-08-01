use crate::error::HwError;

use super::rail_config::RailConfig;

#[cfg(all(windows, feature = "real"))]
pub(super) const MOVE_POLL_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(30);

#[cfg(all(windows, feature = "real"))]
pub(super) struct AxlLive {
    ffi: super::axl_ffi::AxlFfi,
}

#[cfg(all(windows, feature = "real"))]
impl AxlLive {
    pub(super) fn new(ffi: super::axl_ffi::AxlFfi) -> Self {
        return Self { ffi };
    }

    pub(super) fn configure(&mut self, config: &RailConfig) -> Result<(), HwError> {
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
        let pulses =
            i32::try_from(config.pulses_per_meter).map_err(|_| HwError::InvalidConfig {
                reason: format!(
                    "pulses_per_meter={}가 i32 범위를 초과합니다",
                    config.pulses_per_meter
                ),
            })?;
        // 1 board unit = 1 meter = pulses_per_meter pulses (vendor-style Unit=1, Pulse=N).
        check_axl("AxmMotSetMoveUnitPerPulse", unsafe {
            (self.ffi.axm_mot_set_move_unit_per_pulse)(axis, 1.0, pulses)
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

    pub(super) fn read_x_m(&mut self, axis: i32) -> Result<f64, HwError> {
        let mut position_m = 0.0;
        let actual_status = unsafe { (self.ffi.axm_status_get_act_pos)(axis, &mut position_m) };
        if actual_status == super::axl_ffi::AXT_RT_SUCCESS {
            return Ok(position_m);
        }

        let command_status = unsafe { (self.ffi.axm_status_get_cmd_pos)(axis, &mut position_m) };
        if command_status == super::axl_ffi::AXT_RT_SUCCESS {
            return Ok(position_m);
        }
        return Err(read_position_error(actual_status, command_status));
    }

    pub(super) fn start_move_abs_m(
        &mut self,
        config: &RailConfig,
        commanded_m: f64,
        vel: f64,
    ) -> Result<(), HwError> {
        check_axl("AxmMotSetAbsRelMode", unsafe {
            (self.ffi.axm_mot_set_abs_rel_mode)(config.axis, 0)
        })?;
        check_axl("AxmMoveStartPos", unsafe {
            (self.ffi.axm_move_start_pos)(config.axis, commanded_m, vel, config.accel, config.decel)
        })?;
        return Ok(());
    }

    /// 진행 중인 1차 명령을 감속 정지한다.
    /// 호출자는 정지 후 실제 위치를 다시 읽어 2차 속도를 계산한다.
    pub(super) fn stop_for_retarget(&mut self, axis: i32) -> Result<bool, HwError> {
        let mut in_motion = 0;
        check_axl("AxmStatusReadInMotion", unsafe {
            (self.ffi.axm_status_read_in_motion)(axis, &mut in_motion)
        })?;
        if in_motion == 0 {
            return Ok(false);
        }
        check_axl("AxmMoveSStop", unsafe { (self.ffi.axm_move_s_stop)(axis) })?;
        self.wait_idle(axis)?;
        return Ok(true);
    }

    pub(super) fn move_abs_m_blocking(
        &mut self,
        config: &RailConfig,
        commanded_m: f64,
    ) -> Result<(), HwError> {
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
        return Ok(());
    }

    pub(super) fn wait_idle(&mut self, axis: i32) -> Result<(), HwError> {
        let deadline = std::time::Instant::now() + MOVE_POLL_TIMEOUT;
        loop {
            let mut in_motion = 0;
            check_axl("AxmStatusReadInMotion", unsafe {
                (self.ffi.axm_status_read_in_motion)(axis, &mut in_motion)
            })?;
            if in_motion == 0 {
                return Ok(());
            }
            if std::time::Instant::now() >= deadline {
                return Err(move_poll_timeout_error());
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}

#[cfg(all(windows, feature = "real"))]
impl Drop for AxlLive {
    fn drop(&mut self) {
        // 서보는 켠 채로 둔다 — 반복 실행·벤치 세션에서 엔코더/홀딩을 유지한다.
        let _ = unsafe { (self.ffi.axl_close)() };
    }
}

#[cfg(all(windows, feature = "real"))]
pub(super) fn check_axl(name: &str, code: u32) -> Result<(), HwError> {
    if code == super::axl_ffi::AXT_RT_SUCCESS {
        return Ok(());
    }
    tracing::debug!(axl_fn = name, code, "AXL API 실패");
    return Err(HwError::InvalidConfig {
        reason: format!("AXL {name} code={code}"),
    });
}

#[cfg(all(windows, feature = "real"))]
pub(super) fn read_position_error(actual_status: u32, command_status: u32) -> HwError {
    return HwError::InvalidConfig {
        reason: format!(
            "AXL AxmStatusGetActPos code={actual_status}; AxmStatusGetCmdPos code={command_status}"
        ),
    };
}

#[cfg(all(windows, feature = "real"))]
pub(super) fn move_poll_timeout_error() -> HwError {
    return HwError::InvalidConfig {
        reason: format!(
            "AXL AxmStatusReadInMotion timeout after {}s",
            MOVE_POLL_TIMEOUT.as_secs()
        ),
    };
}
