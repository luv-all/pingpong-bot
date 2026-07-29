//! Dynamixel MX 시리즈 datasheet 고정값 (Robotis e-Manual).
//!
//! 연속 토크 derate·관절 속도 상한은 [`crate::defaults::dxl_limits`] (엔지니어링 가정).

/// Protocol 2.0 Profile Velocity 1 LSB [rev/min].
///
/// source: MX-64 / MX-28 Protocol 2.0 control table address 112.
pub const PROFILE_VELOCITY_REV_MIN_PER_LSB: f64 = 0.229;

/// MX-28T 무부하 속도 [rev/min], 12.0V.
pub const MX28_NO_LOAD_SPEED_RPM: f64 = 55.0;

/// 12.0V stall torque [N·m] — MX-64R.
pub const MX64_STALL_TORQUE_NM: f64 = 6.0;

/// 12.0V stall torque [N·m] — MX-28T.
pub const MX28_STALL_TORQUE_NM: f64 = 2.5;

/// MX-64 감속비 N (200:1) — e-Manual "Gear Ratio".
pub const MX64_GEAR_RATIO: f64 = 200.0;

/// MX-28 감속비 N (193:1) — e-Manual "Gear Ratio".
pub const MX28_GEAR_RATIO: f64 = 193.0;

/// rev/min → rad/s.
pub const fn rev_min_to_rad_s(rev_min: f64) -> f64 {
    return rev_min * std::f64::consts::TAU / 60.0;
}
