//! 공 궤적 추정 (탄도학, EKF, 패스스루 스텁).

pub mod ballistics;
pub mod ekf;
pub mod identify;
mod passthrough;

pub use ballistics::predict_hit_plane;
pub use ekf::BallEkf;
pub use identify::{
    drag_from_trajectory, friction_from_tangential_speeds, physics_coeffs_toml,
    restitution_from_bounce_heights, restitution_from_normal_speeds,
};
pub use passthrough::PassThroughEstimator;
