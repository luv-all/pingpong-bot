//! 궤적에서 물리 계수를 식별하는 공개 진입점.

use nalgebra::Vector3;

use super::identify;

pub struct PhysicsIdentify;

impl PhysicsIdentify {
    pub fn restitution_from_bounce_heights(heights: &[f64]) -> Option<f64> {
        return identify::restitution_from_bounce_heights(heights);
    }

    pub fn restitution_from_normal_speeds(pairs: &[(f64, f64)]) -> Option<f64> {
        return identify::restitution_from_normal_speeds(pairs);
    }

    pub fn friction_from_tangential_speeds(pairs: &[(f64, f64)]) -> Option<f64> {
        return identify::friction_from_tangential_speeds(pairs);
    }

    pub fn drag_from_trajectory(samples: &[(f64, Vector3<f64>)]) -> Option<f64> {
        return identify::drag_from_trajectory(samples);
    }

    pub fn format_physics_for_defaults(
        restitution: Option<f64>,
        friction: Option<f64>,
        drag: Option<f64>,
    ) -> String {
        return identify::format_physics_for_defaults(restitution, friction, drag);
    }
}
