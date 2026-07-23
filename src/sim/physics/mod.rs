//! Rapier 물리 월드 — 탁구대·슈터·팔 멀티바디.

pub mod arm_bodies;
mod rapier_convert;
pub mod shooter;
pub mod world;

pub use arm_bodies::ArmMultibody;
pub use shooter::{BallShooterSettings, BallState, ShooterLayout};
pub use world::SimWorld;
