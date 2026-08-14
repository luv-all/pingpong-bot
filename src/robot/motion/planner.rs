//! 스윙 계획의 공개 진입점.

use nalgebra::Vector3;

use crate::Point3;
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

    /// x 보정 없이 예측된 공 위치에, 라켓 중심보다
    /// 0.5cm 아래 지점이 닿도록 정지 정렬한다.
    /// 라켓 면은 공 반지름+라켓 반두께만큼 뒤에 둔다.
    pub fn ball_alignment(
        arm: &Arm,
        start: &robot::Pose,
        ball: crate::Point3,
    ) -> Result<Trajectory, DomainError> {
        return physics::plan_ball_alignment(arm, start, ball);
    }

    /// IK보다 먼저 출발시킬 공별 안전 레일 목표를 계산한다.
    pub fn ball_alignment_rail_target(arm: &Arm, ball: Point3) -> f64 {
        return physics::ball_alignment_rail_target(arm, ball);
    }

    /// 관절은 유지한 채 현재 라켓 중심 x를 공 x에 맞추는 레일 목표.
    pub fn ball_x_tracking_rail_target(arm: &Arm, start: &robot::Pose, ball_x: f64) -> f64 {
        return physics::ball_x_tracking_rail_target(arm, start, ball_x);
    }

    pub fn ball_alignment_rail_target_unclamped(ball: Point3) -> f64 {
        return physics::ball_alignment_rail_target_unclamped(ball);
    }

    pub fn ball_alignment_pose(
        arm: &Arm,
        start: &robot::Pose,
        ball: Point3,
    ) -> Result<robot::Pose, DomainError> {
        return physics::ball_alignment_pose(arm, start, ball);
    }

    pub fn ball_alignment_bearing_error_deg(
        arm: &Arm,
        pose: &robot::Pose,
        ball: Point3,
    ) -> Option<f64> {
        return physics::ball_alignment_bearing_error_deg(arm, pose, ball);
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

    /// 접힌 정렬 자세에서 별도 백스윙 없이 j0~j3로 라켓을 바로 민다 —
    /// 관절 배분은 IK가 정하며 특정 관절의 역할을 강제하지 않는다.
    pub fn fixed_joint_swing(
        arm: &Arm,
        start: &robot::Pose,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing(arm, start);
    }

    /// 실측 시작 자세와 별개로 마지막 정렬 목표를 절대 푸시 기준으로 사용한다.
    pub fn fixed_joint_swing_from_alignment(
        arm: &Arm,
        start: &robot::Pose,
        aligned: &robot::Pose,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_from_alignment(arm, start, aligned);
    }

    /// [`Self::fixed_joint_swing`]의 등가속(quadratic) 버전 — A/B 비교용.
    /// 임팩트 목표 선속도를 고정하지 않고, 목표 위치와 소요시간에서 유일하게
    /// 정해지는 등가속을 그대로 쓴다.
    pub fn fixed_joint_swing_quadratic(
        arm: &Arm,
        start: &robot::Pose,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_quadratic(arm, start);
    }

    /// [`Self::fixed_joint_swing_from_alignment`]의 등가속 버전.
    pub fn fixed_joint_swing_quadratic_from_alignment(
        arm: &Arm,
        start: &robot::Pose,
        aligned: &robot::Pose,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_quadratic_from_alignment(arm, start, aligned);
    }

    /// [`Self::fixed_joint_swing_quadratic`]를 대체하는 파워 스윙 — j0·j2가
    /// 관절 속도 상한까지 가속-순항하며 임팩트를 만들고, j3는 IK 목표와
    /// 무관하게 위쪽 15° 범위 안에서 관절 최고속도까지 가속한 뒤 멈춘다.
    /// `target_impact_time_secs`는 타격-전 전체 시간의 목표값이다
    /// (`FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS` 미만이면 그 값으로
    /// 클램프된다).
    pub fn fixed_joint_swing_power_sweep(
        arm: &Arm,
        start: &robot::Pose,
        target_impact_time_secs: f64,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_power_sweep(arm, start, target_impact_time_secs);
    }

    /// [`Self::fixed_joint_swing_power_sweep`]의 정렬-기준 버전.
    pub fn fixed_joint_swing_power_sweep_from_alignment(
        arm: &Arm,
        start: &robot::Pose,
        aligned: &robot::Pose,
        target_impact_time_secs: f64,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_swing_power_sweep_from_alignment(
            arm,
            start,
            aligned,
            target_impact_time_secs,
        );
    }

    /// 자세 IK 실패 시 레일은 유지하고 j0·j2 고정 밀치기와 j3 상향 전속
    /// 스윙을 합성하는 IK 없는 폴백.
    pub fn fixed_joint_push_fallback(
        arm: &Arm,
        start: &robot::Pose,
        target_impact_time_secs: f64,
    ) -> Result<physics::FixedJointSwing, DomainError> {
        return physics::plan_fixed_joint_push_fallback(arm, start, target_impact_time_secs);
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
