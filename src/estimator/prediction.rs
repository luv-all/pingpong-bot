//! EKF가 낸 임팩트 시점 공 상태.

use nalgebra::Vector3;

use crate::Point3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prediction {
    pub time_to_impact_secs: f64,
    pub impact_position: Point3,
    pub incoming_velocity: Vector3<f64>,
}
