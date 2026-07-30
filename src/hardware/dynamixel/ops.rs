//! Dynamixel 좌표 변환 헬퍼.

use crate::constants::dynamixel::{PROFILE_VELOCITY_REV_MIN_PER_LSB, rev_min_to_rad_s};

/// Dynamixel Protocol 2.0 `Profile Velocity` 레지스터 값 → 관절 각속도 [rad/s].
///
/// 1 LSB = [`PROFILE_VELOCITY_REV_MIN_PER_LSB`] rev/min.
pub fn dynamixel_profile_velocity_to_rad_s(profile_velocity: u32) -> f64 {
    return rev_min_to_rad_s(f64::from(profile_velocity) * PROFILE_VELOCITY_REV_MIN_PER_LSB);
}
