//! 궤적 측정·물리 계수 식별 (e, μ, drag).

mod identify;
mod traj_measure;

pub use identify::{
    drag_from_trajectory, format_physics_for_defaults, friction_from_tangential_speeds,
    restitution_from_bounce_heights, restitution_from_normal_speeds,
};
pub use traj_measure::{
    BounceEvent, RollEvent, TrajPoint, detect_bounces, detect_rolls, mean_bounce_e, mean_roll_mu,
};
