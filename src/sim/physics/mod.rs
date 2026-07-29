//! Rapier 물리 월드 — 탁구대·슈터·팔 멀티바디.

pub mod arm_bodies;
mod bang_bang_worker;
mod rapier_convert;
mod step_input;
pub mod world;

pub use step_input::SimStepInput;

pub use arm_bodies::ArmMultibody;
pub use world::SimWorld;

/// 하위 호환: `sim::physics::shooter::*`
pub mod shooter {
    pub use crate::ball::State;
    pub use crate::defaults::sim::{
        RANDOM_SHOT_HEIGHT_MAX_M, RANDOM_SHOT_HEIGHT_MIN_M, RANDOM_SHOT_LATERAL_MAX_M,
        RANDOM_SHOT_LATERAL_MIN_M, RANDOM_SHOT_NET_GATE_MAX_TRIES, RANDOM_SHOT_PITCH_MAX_DEG,
        RANDOM_SHOT_PITCH_MIN_DEG, RANDOM_SHOT_ROLL_MAX_DEG, RANDOM_SHOT_ROLL_MIN_DEG,
        RANDOM_SHOT_SIDESPIN_MAX, RANDOM_SHOT_SIDESPIN_MIN, RANDOM_SHOT_SPEED_MAX_MPS,
        RANDOM_SHOT_SPEED_MIN_MPS, RANDOM_SHOT_TARGET_PADDING_M, RANDOM_SHOT_TOPSPIN_MAX,
        RANDOM_SHOT_TOPSPIN_MIN,
    };
    pub use crate::shooter::{Layout, Settings};
}
