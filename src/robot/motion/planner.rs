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

    /// [`Self::return_to_center`]과 같지만 목표 레일 x를 호출측이 고른다 —
    /// 좌/센터/우 존 테스트 컨트롤이 쓴다.
    pub fn return_to_center_at(
        arm: &Arm,
        start: &robot::Pose,
        rail_x: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_return_to_center_at(arm, start, rail_x);
    }

    /// [`Self::return_to_center_at`]와 같지만 `speed_ratio`만큼 늦춘 궤적을 계획한다 —
    /// 홈 포지션 복귀·시작 자세 초기화처럼 랠리보다 느려도 되는 이동에 쓴다.
    pub fn return_to_center_at_speed_ratio(
        arm: &Arm,
        start: &robot::Pose,
        rail_x: f64,
        speed_ratio: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_return_to_center_at_speed_ratio(arm, start, rail_x, speed_ratio);
    }

    pub fn ready_prewind(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
        return physics::plan_ready_prewind(arm, start);
    }

    /// 발사기 기준 오른쪽 6cm로 보정한 예측 위치에, 라켓 중심보다
    /// 0.5cm 아래 지점이 닿도록 정지 정렬한다.
    /// 라켓 면은 공 반지름+라켓 반두께만큼 뒤에 둔다.
    pub fn ball_alignment(
        arm: &Arm,
        start: &robot::Pose,
        ball: crate::Point3,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_ball_alignment(arm, start, ball);
    }

    /// 레일은 현재 위치에 고정하고 Dynamixel 관절만 공 예측 위치로 정렬한다.
    pub fn ball_alignment_fixed_rail(
        arm: &Arm,
        start: &robot::Pose,
        ball: crate::Point3,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_ball_alignment_fixed_rail(arm, start, ball);
    }

    pub fn aligned_impact_sequence(
        arm: &Arm,
        start: &robot::Pose,
        ball: crate::Point3,
        time_to_impact_secs: f64,
    ) -> Result<physics::AlignedImpactSequence, DomainError> {
        return physics::plan_aligned_impact_sequence(arm, start, ball, time_to_impact_secs);
    }

    /// 정렬 자세에서 0.1초 고정 관절 스윙을 만든다.
    pub fn fixed_joint_swing(
        arm: &Arm,
        start: &robot::Pose,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing(arm, start);
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

    /// [`Self::move_to`]와 같지만 `speed_ratio`만큼 늦춘 궤적을 계획한다.
    pub fn move_to_at_speed_ratio(
        arm: &Arm,
        start: &robot::Pose,
        end_joints: crate::robot::Joints,
        end_rail_x: f64,
        speed_ratio: f64,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_move_to_at_speed_ratio(
            arm,
            start,
            end_joints,
            end_rail_x,
            speed_ratio,
        );
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
