//! Dynamixel 4축 설정·좌표 변환과 Protocol 2.0 통신.
//!
//! SSOT: `test-manipulator`의 `DynamixelConfig` / `DynamixelController`.
//! - `radians_to_ticks` / `ticks_to_radians` = Python 동일 식
//! - Goal/Torque/Profile SyncWrite = Python `_pack_u32` / `_pack_u8`
//! - `enable_torque(true)` = profile 재적용 → (추가) Goal=Present → Torque ON

mod bus_backend;
mod dynamixel_bus;
mod dynamixel_config;
mod dynamixel_config_error;
mod mirror_slave;
mod motor_mapping;
mod ops;
#[cfg(feature = "real")]
mod real_backend;

#[cfg(test)]
mod tests;

pub use crate::constants::dynamixel::{
    MX28_NO_LOAD_SPEED_RPM, MX28_STALL_TORQUE_NM, MX64_STALL_TORQUE_NM,
    PROFILE_VELOCITY_REV_MIN_PER_LSB, rev_min_to_rad_s,
};
pub use crate::defaults::dxl_limits::{
    CONTINUOUS_TORQUE_DERATE, DYNAMIXEL_MAX_JOINT_SPEED_RAD_S, JOINT_SPEED_DERATE,
    joint_torque_limits_4dof, joint_torque_limits_4dof_array,
};

pub use dynamixel_bus::DynamixelBus;
pub use dynamixel_config::DynamixelConfig;
pub use dynamixel_config_error::DynamixelConfigError;
pub use mirror_slave::MirrorSlave;
pub use motor_mapping::MotorMapping;
pub use ops::dynamixel_profile_velocity_to_rad_s;
