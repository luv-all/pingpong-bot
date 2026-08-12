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
        check_axl("AxmSignalServoOn", unsafe {
            (self.ffi.axm_signal_servo_on)(axis, super::axl_ffi::ENABLE)
        })?;
        let (actual_position_m, command_position_m) = self.read_actual_and_command_m(axis)?;
        let command_minus_actual_m = command_position_m - actual_position_m;
        let soft_limit = config.soft_limit_args_for_command_offset(command_minus_actual_m);
        tracing::info!(
            axis,
            actual_position_m,
            command_position_m,
            command_minus_actual_m,
            soft_limit_selection = soft_limit.selection,
            soft_limit_positive_m = soft_limit.positive_m,
            soft_limit_negative_m = soft_limit.negative_m,
            "AXL ActPos/CmdPos 원점 및 소프트 리밋 진단"
        );
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
        return Ok(());
    }

    pub(super) fn read_actual_and_command_m(&mut self, axis: i32) -> Result<(f64, f64), HwError> {
        let mut actual_position_m = 0.0;
        let mut command_position_m = 0.0;
        let actual_status =
            unsafe { (self.ffi.axm_status_get_act_pos)(axis, &mut actual_position_m) };
        let command_status =
            unsafe { (self.ffi.axm_status_get_cmd_pos)(axis, &mut command_position_m) };
        if actual_status != super::axl_ffi::AXT_RT_SUCCESS
            || command_status != super::axl_ffi::AXT_RT_SUCCESS
        {
            return Err(read_position_error(actual_status, command_status));
        }
        return Ok((actual_position_m, command_position_m));
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
        actual_target_m: f64,
        vel: f64,
    ) -> Result<(), HwError> {
        // 1차 목표로 이동 중 정밀 목표가 오면 기존 명령을 부드럽게 감속
        // 정지한 뒤 새 목표를 건다. 예전처럼 InMotion을 무시하면 관절만
        // 정밀 위치로 가고 레일은 1차 위치로 가는 실기 불일치가 생긴다.
        self.stop_if_moving(config.axis)?;

        let (actual_now_m, command_now_m) = self.read_actual_and_command_m(config.axis)?;
        let command_target_m = RailConfig::command_position_for_actual_target(
            actual_target_m,
            actual_now_m,
            command_now_m,
        );
        tracing::info!(
            axis = config.axis,
            actual_now_m,
            command_now_m,
            command_minus_actual_m = command_now_m - actual_now_m,
            actual_target_m,
            command_target_m,
            "AXL ActPos 기준 절대 목표 보정"
        );

        check_axl("AxmMotSetAbsRelMode", unsafe {
            (self.ffi.axm_mot_set_abs_rel_mode)(config.axis, 0)
        })?;
        check_axl("AxmMoveStartPos", unsafe {
            (self.ffi.axm_move_start_pos)(
                config.axis,
                command_target_m,
                vel,
                config.accel,
                config.decel,
            )
        })?;
        return Ok(());
    }

    pub(super) fn stop_if_moving(&mut self, axis: i32) -> Result<(), HwError> {
        let mut in_motion = 0;
        check_axl("AxmStatusReadInMotion", unsafe {
            (self.ffi.axm_status_read_in_motion)(axis, &mut in_motion)
        })?;
        if in_motion != 0 {
            self.stop(axis)?;
            self.wait_idle(axis)?;
        }
        return Ok(());
    }

    pub(super) fn move_abs_m_blocking(
        &mut self,
        config: &RailConfig,
        actual_target_m: f64,
    ) -> Result<(), HwError> {
        let (actual_now_m, command_now_m) = self.read_actual_and_command_m(config.axis)?;
        let command_target_m = RailConfig::command_position_for_actual_target(
            actual_target_m,
            actual_now_m,
            command_now_m,
        );
        tracing::info!(
            axis = config.axis,
            actual_now_m,
            command_now_m,
            command_minus_actual_m = command_now_m - actual_now_m,
            actual_target_m,
            command_target_m,
            "AXL ActPos 기준 블로킹 목표 보정"
        );
        check_axl("AxmMotSetAbsRelMode", unsafe {
            (self.ffi.axm_mot_set_abs_rel_mode)(config.axis, 0)
        })?;
        check_axl("AxmMovePos", unsafe {
            (self.ffi.axm_move_pos)(
                config.axis,
                command_target_m,
                config.vel,
                config.accel,
                config.decel,
            )
        })?;
        return Ok(());
    }

    pub(super) fn read_alarm(&mut self, axis: i32) -> Result<bool, HwError> {
        let mut alarm = 0;
        check_axl("AxmSignalReadServoAlarm", unsafe {
            (self.ffi.axm_signal_read_servo_alarm)(axis, &mut alarm)
        })?;
        return Ok(alarm != 0);
    }

    pub(super) fn reset_alarm(&mut self, axis: i32) -> Result<(), HwError> {
        const LOW: u32 = 0;
        const HIGH: u32 = 1;
        check_axl("AxmSignalServoAlarmReset", unsafe {
            (self.ffi.axm_signal_servo_alarm_reset)(axis, LOW)
        })?;
        std::thread::sleep(std::time::Duration::from_millis(500));
        check_axl("AxmSignalServoAlarmReset", unsafe {
            (self.ffi.axm_signal_servo_alarm_reset)(axis, HIGH)
        })?;
        let deadline = std::time::Instant::now() + MOVE_POLL_TIMEOUT;
        loop {
            if !self.read_alarm(axis)? {
                break;
            }
            if std::time::Instant::now() >= deadline {
                return Err(HwError::InvalidConfig {
                    reason: "AXL 알람 해제 실패 — AxmSignalReadServoAlarm이 계속 true, 수동 확인 필요"
                        .into(),
                });
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        return check_axl("AxmSignalServoAlarmReset", unsafe {
            (self.ffi.axm_signal_servo_alarm_reset)(axis, LOW)
        });
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

    pub(super) fn stop(&mut self, axis: i32) -> Result<(), HwError> {
        return check_axl("AxmMoveSStop", unsafe { (self.ffi.axm_move_s_stop)(axis) });
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
