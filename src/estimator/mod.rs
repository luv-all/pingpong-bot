//! 공 궤적 추정 (탄도학, EKF).

use std::time::Instant;

use nalgebra::Vector3;

use crate::Point3;

pub mod ballistics;
pub mod ekf;
pub mod identify;
pub mod traj_measure;

pub use ballistics::{clears_net_gate, predict_hit_plane, semi_implicit_euler};
pub use ekf::BallEkf;
pub use identify::{
    drag_from_trajectory, format_physics_for_defaults, friction_from_tangential_speeds,
    restitution_from_bounce_heights, restitution_from_normal_speeds,
};
pub use traj_measure::{
    BounceEvent, RollEvent, TrajPoint, detect_bounces, detect_rolls, mean_bounce_e, mean_roll_mu,
};

/// 접수 평면. 월드 y [m] 하나.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitPlane {
    pub y: f64,
}

/// EKF가 낸 임팩트 시점 공 상태.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prediction {
    pub time_to_impact_secs: f64,
    pub impact_position: Point3,
    pub incoming_velocity: Vector3<f64>,
}

/// 공 상태 추정과 타격 평면 예측.
pub trait Estimator: Send {
    fn update(&mut self, position: Point3, timestamp: Instant);
    fn predict_to(&self, plane: HitPlane) -> Option<Prediction>;
}
