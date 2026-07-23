//! 스윙/충돌/임팩트/관절 궤적 계획.

pub mod collision;
pub mod dynamics;
pub mod impact;
pub mod physics;
pub mod trajectory;

pub use collision::{OrientedBox, clamp_above_table, robot_obbs, table_penetration};
pub use impact::{rally_return_velocity, required_racket_velocity, verify_impact_model};
pub use physics::{
    PlannedIntercept, accel, ball_past_midcourt_for_commit, in_swing_commit_window,
    plan_best_swing, plan_coarse_track, plan_return_to_center, plan_swing,
};
pub use trajectory::{RailMotion, SwingTrajectory};

use crate::estimator::HitPlane;

/// 로봇 앞에서 탐색할 동적 인터셉트 y 구간.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InterceptWindow {
    pub y_min: f64,
    pub y_max: f64,
    pub sample_step: f64,
}

pub const MAX_INTERCEPT_SAMPLES: usize = 1_024;

impl InterceptWindow {
    pub fn hit_planes(self) -> Vec<HitPlane> {
        if !self.y_min.is_finite()
            || !self.y_max.is_finite()
            || !self.sample_step.is_finite()
            || self.y_min > self.y_max
            || self.sample_step <= 0.0
        {
            return Vec::new();
        }
        let intervals = ((self.y_max - self.y_min) / self.sample_step).ceil();
        if !intervals.is_finite() || intervals + 1.0 > MAX_INTERCEPT_SAMPLES as f64 {
            return Vec::new();
        }
        let intervals = intervals as usize;
        let mut planes = Vec::with_capacity(intervals + 1);
        for index in 0..intervals {
            planes.push(HitPlane {
                y: self.y_min + self.sample_step * index as f64,
            });
        }
        if planes
            .last()
            .is_none_or(|plane| (plane.y - self.y_max).abs() > 1e-12)
        {
            planes.push(HitPlane { y: self.y_max });
        }
        return planes;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intercept_window_samples_both_bounds() {
        let window = InterceptWindow {
            y_min: 0.20,
            y_max: 0.50,
            sample_step: 0.10,
        };
        let ys: Vec<f64> = window
            .hit_planes()
            .into_iter()
            .map(|plane| plane.y)
            .collect();
        assert_eq!(ys.len(), 4);
        for (actual, expected) in ys.iter().zip([0.20, 0.30, 0.40, 0.50]) {
            assert!((actual - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn intercept_window_rejects_unbounded_sample_count() {
        let window = InterceptWindow {
            y_min: 0.20,
            y_max: 0.50,
            sample_step: 1e-20,
        };
        assert!(window.hit_planes().is_empty());
    }
}
