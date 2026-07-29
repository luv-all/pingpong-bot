//! 공 궤적 추정 (탄도학, EKF).

use std::time::Instant;

use nalgebra::Vector3;

use crate::Point3;
use crate::defaults::PhysicsParams;

pub mod ballistics;
pub mod bounce;
pub mod ekf;
pub mod measure;

pub use ekf::BallEkf;
pub use measure::{BounceEvent, PhysicsIdentify, RollEvent, TrajAnalysis, TrajPoint};

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

/// 공 탄도·바운스 예측의 공개 진입점.
pub struct BallKinematics;

impl BallKinematics {
    pub fn clears_net(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        physics: &PhysicsParams,
    ) -> bool {
        return ballistics::clears_net_gate(position, velocity, omega, physics);
    }

    pub fn predict_to(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        plane: HitPlane,
        physics: &PhysicsParams,
    ) -> Option<Prediction> {
        return ballistics::predict_hit_plane(position, velocity, omega, plane, physics);
    }

    pub fn step(
        pos: Vector3<f64>,
        vel: Vector3<f64>,
        omega: Vector3<f64>,
        dt: f64,
        physics: &PhysicsParams,
    ) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        return ballistics::semi_implicit_euler(pos, vel, omega, dt, physics);
    }

    pub fn table_ball_mu(physics: &PhysicsParams) -> f64 {
        return bounce::table_ball_mu(physics);
    }

    pub fn rapier_table_ball_mu(physics: &PhysicsParams) -> f64 {
        return bounce::rapier_table_ball_mu(physics);
    }

    pub fn bounce_on_table(
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        physics: &PhysicsParams,
    ) -> (Vector3<f64>, Vector3<f64>) {
        return bounce::table_bounce(velocity, omega, physics);
    }
}
