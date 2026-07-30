//! 로봇 조립 — Arm/Robot 빌더.

mod arm_build_error;
mod arm_builder;
mod build_error;
mod mount_preset;
mod robot;
mod robot_builder;

pub mod builder;
pub mod loader;

pub use arm_build_error::ArmBuildError;
pub use arm_builder::ArmBuilder;
pub use build_error::RobotBuildError;
pub use mount_preset::MountPreset;
pub use robot::Robot;
pub use robot_builder::RobotBuilder;
