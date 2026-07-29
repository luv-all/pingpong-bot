//! 궤적 측정·물리 계수 식별 (e, μ, drag).

mod identify;
mod traj_measure;

use nalgebra::Vector3;

pub use traj_measure::{BounceEvent, RollEvent, TrajPoint};

/// 궤적에서 물리 계수를 식별하는 공개 진입점.
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

/// 삼각측량 궤적의 이벤트 분석 공개 진입점.
pub struct TrajAnalysis;

impl TrajAnalysis {
    pub fn detect_bounces(traj: &[TrajPoint]) -> Vec<BounceEvent> {
        return traj_measure::detect_bounces(traj);
    }

    pub fn detect_rolls(traj: &[TrajPoint]) -> Vec<RollEvent> {
        return traj_measure::detect_rolls(traj);
    }

    pub fn mean_bounce_e(events: &[BounceEvent]) -> Option<f64> {
        return traj_measure::mean_bounce_e(events);
    }

    pub fn mean_roll_mu(events: &[RollEvent]) -> Option<f64> {
        return traj_measure::mean_roll_mu(events);
    }
}
