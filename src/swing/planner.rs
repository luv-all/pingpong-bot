//! 스윙 계획의 공개 진입점.

use nalgebra::Vector3;

use crate::error::DomainError;
use crate::estimator::Prediction;
use crate::robot::{self, Arm};

use super::bang_bang::{self, PlannedIntercept as BangBangPlannedIntercept};
use super::feasibility::Feasibility;
use super::physics;
use super::planned_intercept::PlannedIntercept;
use super::trajectory::Trajectory;

/// 스윙 계획의 공개 진입점.
pub struct Planner;

impl Planner {
    pub fn aero_accel(
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        drag_coefficient: f64,
        magnus_coefficient: f64,
    ) -> Vector3<f64> {
        return physics::aero_accel(velocity, omega, drag_coefficient, magnus_coefficient);
    }

    pub fn in_commit_window(time_to_impact_secs: f64) -> bool {
        return physics::in_swing_commit_window(time_to_impact_secs);
    }

    pub fn past_midcourt(ball_y: f64) -> bool {
        return physics::ball_past_midcourt_for_commit(ball_y);
    }

    pub fn feasibility(
        arm: &Arm,
        prediction: &Prediction,
        start: &robot::Pose,
    ) -> Option<Feasibility> {
        return super::feasibility::feasibility(arm, prediction, start);
    }

    pub fn plan(
        arm: &Arm,
        prediction: Prediction,
        start: &robot::Pose,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_swing(arm, prediction, start);
    }

    pub fn plan_best(
        arm: &Arm,
        predictions: &[Prediction],
        start: &robot::Pose,
    ) -> Result<PlannedIntercept, DomainError> {
        return physics::plan_best_swing(arm, predictions, start);
    }

    pub fn plan_coarse_track(arm: &Arm, predictions: &[Prediction]) -> Option<robot::Pose> {
        return physics::plan_coarse_track(arm, predictions);
    }

    pub fn return_to_center(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
        return physics::plan_return_to_center(arm, start);
    }

    pub fn plan_bang_bang(
        arm: &Arm,
        predictions: &[Prediction],
        start: &robot::Pose,
    ) -> Result<BangBangPlannedIntercept, DomainError> {
        return bang_bang::plan_bang_bang_swing(arm, predictions, start);
    }

    pub fn step_racket_guidance(
        arm: &Arm,
        q: &mut [f64],
        qdot: &mut [f64],
        rail_x: &mut f64,
        rail_v: &mut f64,
        target_racket_position: Vector3<f64>,
        target_racket_velocity: Vector3<f64>,
        remaining_secs: f64,
        dt: f64,
        scratch: &mut bang_bang::RacketGuidanceScratch,
    ) -> Option<bang_bang::RacketGuidanceStep> {
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
