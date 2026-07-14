//! 스윙/충돌/임팩트/관절 궤적 계획.

pub mod collision;
pub mod impact;
pub mod physics;
pub mod trajectory;

pub use collision::{OrientedBox, clamp_above_table, robot_obbs, table_penetration};
pub use impact::{loft_return_velocity, required_racket_velocity, verify_impact_model};
pub use physics::{accel, ball_past_midcourt_for_commit, in_swing_commit_window, plan_swing};
