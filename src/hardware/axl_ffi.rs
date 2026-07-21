//! Windows AXL 동적 라이브러리 바인딩.

use libloading::Library;

use crate::HwError;

pub const AXT_RT_SUCCESS: u32 = 0;
pub const STATUS_EXIST: u32 = 1;
pub const ENABLE: u32 = 1;

type AxlOpen = unsafe extern "system" fn(i32) -> u32;
type AxlClose = unsafe extern "system" fn() -> i32;
type AxmInfoIsMotionModule = unsafe extern "system" fn(*mut u32) -> u32;
type AxmMotSetPulseOutMethod = unsafe extern "system" fn(i32, u32) -> u32;
type AxmMotSetEncInputMethod = unsafe extern "system" fn(i32, u32) -> u32;
type AxmMotSetMoveUnitPerPulse = unsafe extern "system" fn(i32, f64, i32) -> u32;
type AxmMotSetMinVel = unsafe extern "system" fn(i32, f64) -> u32;
type AxmMotSetMaxVel = unsafe extern "system" fn(i32, f64) -> u32;
type AxmMotSetAbsRelMode = unsafe extern "system" fn(i32, u32) -> u32;
type AxmMotSetProfileMode = unsafe extern "system" fn(i32, u32) -> u32;
type AxmMotSetAccelUnit = unsafe extern "system" fn(i32, u32) -> u32;
type AxmSignalSetInpos = unsafe extern "system" fn(i32, u32) -> u32;
type AxmSignalSetServoAlarm = unsafe extern "system" fn(i32, u32) -> u32;
type AxmSignalSetLimit = unsafe extern "system" fn(i32, u32, u32, u32) -> u32;
type AxmSignalSetSoftLimit = unsafe extern "system" fn(i32, u32, u32, u32, f64, f64) -> u32;
type AxmSignalServoOn = unsafe extern "system" fn(i32, u32) -> u32;
type AxmStatusGetActPos = unsafe extern "system" fn(i32, *mut f64) -> u32;
type AxmStatusGetCmdPos = unsafe extern "system" fn(i32, *mut f64) -> u32;
type AxmStatusReadInMotion = unsafe extern "system" fn(i32, *mut u32) -> u32;
type AxmMovePos = unsafe extern "system" fn(i32, f64, f64, f64, f64) -> u32;

/// AXL 헤더와 동일한 stdcall 심볼 테이블. `library`는 함수 포인터 수명 동안 유지된다.
pub struct AxlFfi {
    _library: Library,
    pub axl_open: AxlOpen,
    pub axl_close: AxlClose,
    pub axm_info_is_motion_module: AxmInfoIsMotionModule,
    pub axm_mot_set_pulse_out_method: AxmMotSetPulseOutMethod,
    pub axm_mot_set_enc_input_method: AxmMotSetEncInputMethod,
    pub axm_mot_set_move_unit_per_pulse: AxmMotSetMoveUnitPerPulse,
    pub axm_mot_set_min_vel: AxmMotSetMinVel,
    pub axm_mot_set_max_vel: AxmMotSetMaxVel,
    pub axm_mot_set_abs_rel_mode: AxmMotSetAbsRelMode,
    pub axm_mot_set_profile_mode: AxmMotSetProfileMode,
    pub axm_mot_set_accel_unit: AxmMotSetAccelUnit,
    pub axm_signal_set_inpos: AxmSignalSetInpos,
    pub axm_signal_set_servo_alarm: AxmSignalSetServoAlarm,
    pub axm_signal_set_limit: AxmSignalSetLimit,
    pub axm_signal_set_soft_limit: AxmSignalSetSoftLimit,
    pub axm_signal_servo_on: AxmSignalServoOn,
    pub axm_status_get_act_pos: AxmStatusGetActPos,
    pub axm_status_get_cmd_pos: AxmStatusGetCmdPos,
    pub axm_status_read_in_motion: AxmStatusReadInMotion,
    pub axm_move_pos: AxmMovePos,
}

impl AxlFfi {
    pub fn load(path: &std::path::Path) -> Result<Self, HwError> {
        let library = unsafe { Library::new(path) }.map_err(|error| HwError::InvalidConfig {
            reason: format!("AXL DLL 로드 실패: {error}"),
        })?;

        unsafe {
            return Ok(Self {
                axl_open: *library.get(b"AxlOpen\0").map_err(symbol_error)?,
                axl_close: *library.get(b"AxlClose\0").map_err(symbol_error)?,
                axm_info_is_motion_module: *library
                    .get(b"AxmInfoIsMotionModule\0")
                    .map_err(symbol_error)?,
                axm_mot_set_pulse_out_method: *library
                    .get(b"AxmMotSetPulseOutMethod\0")
                    .map_err(symbol_error)?,
                axm_mot_set_enc_input_method: *library
                    .get(b"AxmMotSetEncInputMethod\0")
                    .map_err(symbol_error)?,
                axm_mot_set_move_unit_per_pulse: *library
                    .get(b"AxmMotSetMoveUnitPerPulse\0")
                    .map_err(symbol_error)?,
                axm_mot_set_min_vel: *library.get(b"AxmMotSetMinVel\0").map_err(symbol_error)?,
                axm_mot_set_max_vel: *library.get(b"AxmMotSetMaxVel\0").map_err(symbol_error)?,
                axm_mot_set_abs_rel_mode: *library
                    .get(b"AxmMotSetAbsRelMode\0")
                    .map_err(symbol_error)?,
                axm_mot_set_profile_mode: *library
                    .get(b"AxmMotSetProfileMode\0")
                    .map_err(symbol_error)?,
                axm_mot_set_accel_unit: *library
                    .get(b"AxmMotSetAccelUnit\0")
                    .map_err(symbol_error)?,
                axm_signal_set_inpos: *library.get(b"AxmSignalSetInpos\0").map_err(symbol_error)?,
                axm_signal_set_servo_alarm: *library
                    .get(b"AxmSignalSetServoAlarm\0")
                    .map_err(symbol_error)?,
                axm_signal_set_limit: *library.get(b"AxmSignalSetLimit\0").map_err(symbol_error)?,
                axm_signal_set_soft_limit: *library
                    .get(b"AxmSignalSetSoftLimit\0")
                    .map_err(symbol_error)?,
                axm_signal_servo_on: *library.get(b"AxmSignalServoOn\0").map_err(symbol_error)?,
                axm_status_get_act_pos: *library
                    .get(b"AxmStatusGetActPos\0")
                    .map_err(symbol_error)?,
                axm_status_get_cmd_pos: *library
                    .get(b"AxmStatusGetCmdPos\0")
                    .map_err(symbol_error)?,
                axm_status_read_in_motion: *library
                    .get(b"AxmStatusReadInMotion\0")
                    .map_err(symbol_error)?,
                axm_move_pos: *library.get(b"AxmMovePos\0").map_err(symbol_error)?,
                _library: library,
            });
        }
    }
}

fn symbol_error(error: libloading::Error) -> HwError {
    return HwError::InvalidConfig {
        reason: format!("AXL DLL 심볼 로드 실패: {error}"),
    };
}
