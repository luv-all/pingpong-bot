//! 공 탄도·바운스 예측의 공개 진입점.

use nalgebra::Vector3;

use crate::defaults::PhysicsParams;
use crate::estimator::{HitPlane, Prediction};

use super::{ballistics, bounce};

pub struct Kinematics;

impl Kinematics {
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
