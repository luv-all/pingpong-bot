//! 스윙/충돌/임팩트/관절 궤적 계획.

pub mod bang_bang;
pub mod collision;
pub mod impact;
pub mod swing;

pub use bang_bang::{
    BangBangTrajectory, PlannedBangBangIntercept, RacketGuidanceScratch, RacketGuidanceStep,
    plan_bang_bang_swing, step_racket_guidance,
};
pub use collision::{OrientedBox, clamp_above_table, robot_obbs, table_penetration};
pub use impact::{rally_return_velocity, required_racket_velocity, verify_impact_model};
/// 하위 호환: `planner::physics::…`
pub use swing::physics;
/// 하위 호환: `planner::trajectory::…`
pub use swing::trajectory;
pub use swing::{
    PlannedIntercept, RailMotion, SwingFeasibility, SwingTrajectory, accel, aero_accel,
    ball_past_midcourt_for_commit, in_swing_commit_window, plan_best_swing, plan_coarse_track,
    plan_coarse_track_targets, plan_return_to_center, plan_swing, swing_feasibility,
};

use anyhow::{Result, ensure};

use crate::estimator::HitPlane;

/// 로봇 앞에서 탐색할 동적 인터셉트 y 구간.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InterceptWindow {
    pub y_min: f64,
    pub y_max: f64,
    pub sample_step: f64,
}

pub use crate::defaults::planner::MAX_INTERCEPT_SAMPLES;

impl InterceptWindow {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.y_min.is_finite(), "y_min finite");
        ensure!(self.y_max.is_finite(), "y_max finite");
        ensure!(self.sample_step.is_finite(), "sample_step finite");
        ensure!(self.y_min <= self.y_max, "y_min <= y_max");
        ensure!(self.sample_step > 0.0, "sample_step > 0");
        let intervals = ((self.y_max - self.y_min) / self.sample_step).ceil();
        ensure!(
            intervals.is_finite() && intervals + 1.0 <= MAX_INTERCEPT_SAMPLES as f64,
            "intercept samples <= {MAX_INTERCEPT_SAMPLES}"
        );
        return Ok(());
    }

    pub fn hit_planes(self) -> Vec<HitPlane> {
        if self.validate().is_err() {
            return Vec::new();
        }
        let intervals = ((self.y_max - self.y_min) / self.sample_step).ceil() as usize;
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
