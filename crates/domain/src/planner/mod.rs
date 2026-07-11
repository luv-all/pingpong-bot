//! 스윙·충돌·임팩트·관절 궤적 계획.

pub mod collision;
pub mod impact;
pub mod physics;
pub mod trajectory;

pub use collision::{
    OrientedBox, clamp_above_table, robot_obbs, robot_obbs_all, table_penetration,
};
pub use impact::{
    RacketImpactTarget, cooperative_return_velocity, loft_return_velocity,
    required_racket_velocity, verify_impact_model,
};
pub use physics::{
    accel, in_swing_commit_window, plan_contact_swing, plan_swing, verify_torque_limits,
};
