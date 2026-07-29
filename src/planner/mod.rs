//! 스윙/충돌/임팩트/관절 궤적 계획.

pub mod bang_bang;
pub mod collision;
pub mod impact;
pub mod swing;

pub use bang_bang::{
    BangBangTrajectory, PlannedBangBangIntercept, RacketGuidanceScratch, RacketGuidanceStep,
};
pub use collision::OrientedBox;
pub use swing::{PlannedIntercept, RailMotion, SwingFeasibility, SwingTrajectory};

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

/// 임팩트 역산의 공개 진입점.
pub struct Impact;

impl Impact {
    pub fn rally_return(
        impact: crate::Point3,
        incoming_velocity: nalgebra::Vector3<f64>,
    ) -> nalgebra::Vector3<f64> {
        return impact::rally_return_velocity(impact, incoming_velocity);
    }

    pub fn required_racket_velocity(
        incoming_velocity: nalgebra::Vector3<f64>,
        outgoing_velocity: nalgebra::Vector3<f64>,
        normal: nalgebra::Vector3<f64>,
        restitution: f64,
    ) -> Result<nalgebra::Vector3<f64>, crate::error::SwingPlanError> {
        return impact::required_racket_velocity(
            incoming_velocity,
            outgoing_velocity,
            normal,
            restitution,
        );
    }

    pub fn verify(
        incoming_velocity: nalgebra::Vector3<f64>,
        outgoing_velocity: nalgebra::Vector3<f64>,
        racket_velocity: nalgebra::Vector3<f64>,
        normal: nalgebra::Vector3<f64>,
        restitution: f64,
    ) -> bool {
        return impact::verify_impact_model(
            incoming_velocity,
            outgoing_velocity,
            racket_velocity,
            normal,
            restitution,
        );
    }

    pub fn clears_net(impact: crate::Point3, outgoing_velocity: nalgebra::Vector3<f64>) -> bool {
        return impact::clears_net_ballistic(impact, outgoing_velocity);
    }
}

/// 스윙 계획의 공개 진입점.
pub struct SwingPlanner;

impl SwingPlanner {
    pub fn aero_accel(
        velocity: nalgebra::Vector3<f64>,
        omega: nalgebra::Vector3<f64>,
        drag_coefficient: f64,
        magnus_coefficient: f64,
    ) -> nalgebra::Vector3<f64> {
        return swing::physics::aero_accel(velocity, omega, drag_coefficient, magnus_coefficient);
    }

    pub fn in_commit_window(time_to_impact_secs: f64) -> bool {
        return swing::physics::in_swing_commit_window(time_to_impact_secs);
    }

    pub fn past_midcourt(ball_y: f64) -> bool {
        return swing::physics::ball_past_midcourt_for_commit(ball_y);
    }

    pub fn feasibility(
        arm: &crate::robot::Arm,
        prediction: &crate::Prediction,
        start: &crate::RobotPose,
    ) -> Option<SwingFeasibility> {
        return swing::physics::swing_feasibility(arm, prediction, start);
    }

    pub fn plan(
        arm: &crate::robot::Arm,
        prediction: crate::Prediction,
        start: &crate::RobotPose,
    ) -> Result<SwingTrajectory, crate::error::DomainError> {
        return swing::physics::plan_swing(arm, prediction, start);
    }

    pub fn plan_best(
        arm: &crate::robot::Arm,
        predictions: &[crate::Prediction],
        start: &crate::RobotPose,
    ) -> Result<PlannedIntercept, crate::error::DomainError> {
        return swing::physics::plan_best_swing(arm, predictions, start);
    }

    pub fn plan_coarse_track(
        arm: &crate::robot::Arm,
        predictions: &[crate::Prediction],
    ) -> Option<crate::RobotPose> {
        return swing::physics::plan_coarse_track(arm, predictions);
    }

    pub fn return_to_center(
        arm: &crate::robot::Arm,
        start: &crate::RobotPose,
    ) -> Result<SwingTrajectory, crate::error::DomainError> {
        return swing::physics::plan_return_to_center(arm, start);
    }

    pub fn plan_bang_bang(
        arm: &crate::robot::Arm,
        predictions: &[crate::Prediction],
        start: &crate::RobotPose,
    ) -> Result<PlannedBangBangIntercept, crate::error::DomainError> {
        return bang_bang::plan_bang_bang_swing(arm, predictions, start);
    }

    pub fn step_racket_guidance(
        arm: &crate::robot::Arm,
        q: &mut [f64],
        qdot: &mut [f64],
        rail_x: &mut f64,
        rail_v: &mut f64,
        target_racket_position: nalgebra::Vector3<f64>,
        target_racket_velocity: nalgebra::Vector3<f64>,
        remaining_secs: f64,
        dt: f64,
        scratch: &mut RacketGuidanceScratch,
    ) -> Option<RacketGuidanceStep> {
        return bang_bang::step_racket_guidance(
            arm,
            q,
            qdot,
            rail_x,
            rail_v,
            target_racket_position,
            target_racket_velocity,
            remaining_secs,
            dt,
            scratch,
        );
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
