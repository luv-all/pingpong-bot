//! 스윙 계획의 공개 진입점.

use nalgebra::Vector3;

use crate::error::DomainError;
use crate::robot::motion::Prediction;
use crate::robot::{self, Arm};

use super::bang_bang::{self, PlannedIntercept as BangBangPlannedIntercept};
use super::feasibility::Feasibility;
use super::physics;
use super::planned_intercept::PlannedIntercept;
use super::trajectory::Trajectory;

/// 스윙 계획의 공개 진입점.
pub struct Planner;

impl Planner {
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

    pub fn plan_coarse_track_targets(
        arm: &Arm,
        predictions: &[Prediction],
    ) -> Option<(f64, Option<robot::Joints>)> {
        return physics::plan_coarse_track_targets(arm, predictions);
    }

    /// coarse 추종이 선호할 평면의 y를 최종 커밋과 같은 WP2b 점수로 고른다.
    /// 평면마다 IK가 필요해 비싸다 — 스로틀된 주기로만 부르고
    /// [`Planner::plan_coarse_track_targets_for_plane`]에 캐시해 넘길 것.
    pub fn best_scored_coarse_plane_y(
        arm: &Arm,
        predictions: &[Prediction],
        start: &robot::Pose,
    ) -> Option<f64> {
        return physics::best_scored_coarse_plane_y(arm, predictions, start);
    }

    /// [`Planner::plan_coarse_track_targets`]와 같은 비용(매 틱 IK 1회)이지만
    /// `preferred_y`(있으면, [`Planner::best_scored_coarse_plane_y`]가 고른 값)에
    /// 가장 가까운 평면을 쫓는다. `None`이면 기존 로봇-최근접 기하로 폴백한다.
    pub fn plan_coarse_track_targets_for_plane(
        arm: &Arm,
        predictions: &[Prediction],
        preferred_y: Option<f64>,
    ) -> Option<(f64, Option<robot::Joints>)> {
        return physics::plan_coarse_track_targets_for_plane(arm, predictions, preferred_y);
    }

    pub fn return_to_center(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
        return physics::plan_return_to_center(arm, start);
    }

    pub fn ready_prewind(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
        return physics::plan_ready_prewind(arm, start);
    }

    /// 타격 없이 라켓 중심을 공 위치에 정지 정렬한다.
    pub fn ball_alignment(
        arm: &Arm,
        start: &robot::Pose,
        ball: crate::Point3,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_ball_alignment(arm, start, ball);
    }

    /// 레일·팔 정렬 중 원래 예측 공 위치에서 짧게 타격한다.
    pub fn ball_alignment_strike(
        arm: &Arm,
        start: &robot::Pose,
        ball: crate::Point3,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_ball_alignment_strike(arm, start, ball);
    }

    pub fn aligned_impact_sequence(
        arm: &Arm,
        start: &robot::Pose,
        ball: crate::Point3,
        time_to_impact_secs: f64,
    ) -> Result<physics::AlignedImpactSequence, DomainError> {
        return physics::plan_aligned_impact_sequence(arm, start, ball, time_to_impact_secs);
    }

    /// 발사기 반복 시험용 짧은 고정 임팩트 푸시.
    pub fn fixed_impact_push(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
        return physics::plan_fixed_impact_push(arm, start);
    }

    /// 지금 관절 동작을 시작해 `impact_duration_secs` 뒤 임팩트에 도달한다.
    pub fn fixed_impact_push_in(
        arm: &Arm,
        start: &robot::Pose,
        impact_duration_secs: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_fixed_impact_push_in(arm, start, impact_duration_secs);
    }

    /// 정지 → 정지로 임의 포즈까지 잇는 최단 실행가능 궤적 (coarse 선추종용).
    pub fn move_to(
        arm: &Arm,
        start: &robot::Pose,
        end_joints: crate::robot::Joints,
        end_rail_x: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_move_to(arm, start, end_joints, end_rail_x);
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
