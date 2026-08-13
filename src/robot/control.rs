//! 공 궤적에서 목표를 고르고 라켓을 그 위치까지 옮기는 공통 제어 경계.

use nalgebra::Vector3;
use thiserror::Error;

use crate::Point3;
use crate::error::DomainError;
use crate::estimator::{BallTrajectory, TrajectorySample};

use super::motion::{Planner, Trajectory};
use super::{Arm, Joints, Pose};

/// 위치 제어기가 받는 유일한 명령.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    /// 라켓 중심의 월드 좌표 [m].
    pub position: Point3,
    /// `BallTrajectory::reference_time`부터 목표 도착까지의 시간 [s].
    pub arrival_time_secs: f64,
}

/// 궤적 선택기가 보존하는 공 상태. 위치 제어기는 [`Self::target`]만 사용한다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitTarget {
    pub position: Point3,
    pub incoming_velocity: Vector3<f64>,
    pub time_secs: f64,
}

impl HitTarget {
    pub fn target(self) -> Target {
        return Target {
            position: self.position,
            arrival_time_secs: self.time_secs,
        };
    }
}

/// `N×7` 미래 궤적만 읽어 하나의 위치/시각을 고른다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitTargetSelector {
    pub y_min: f64,
    pub y_max: f64,
}

impl HitTargetSelector {
    pub fn new(y_min: f64, y_max: f64) -> Result<Self, TargetSelectionError> {
        if !y_min.is_finite() || !y_max.is_finite() || y_min > y_max {
            return Err(TargetSelectionError::InvalidWindow);
        }
        return Ok(Self { y_min, y_max });
    }

    /// 작업 구간 중앙 y를 지나는 점을 선택한다. 중앙을 건너뛰는 샘플 사이에서는
    /// 위치·속도·시간을 같은 비율로 선형 보간한다.
    pub fn select(&self, trajectory: &BallTrajectory) -> Result<HitTarget, TargetSelectionError> {
        let samples = &trajectory.predicted;
        if samples.is_empty() {
            return Err(TargetSelectionError::EmptyPrediction);
        }
        let target_y = (self.y_min + self.y_max) * 0.5;
        if let Some(target) = interpolate_at_y(samples, target_y) {
            return Ok(target);
        }

        // 궤적이 중앙까지 오기 전에 끝났더라도 작업 구간 안의 실제 행은 사용할 수 있다.
        return samples
            .iter()
            .filter(|sample| sample.position.y >= self.y_min && sample.position.y <= self.y_max)
            .min_by(|left, right| {
                (left.position.y - target_y)
                    .abs()
                    .total_cmp(&(right.position.y - target_y).abs())
            })
            .copied()
            .map(HitTarget::from)
            .ok_or(TargetSelectionError::OutsideWindow);
    }

    /// 현재 라켓 위치에서 이동 부담이 작고 준비 시간이 긴 후보부터 반환한다.
    /// 후보는 예측 행과 구간 경계/중앙의 인접 행 보간값으로만 구성한다.
    pub fn ranked_candidates(
        &self,
        trajectory: &BallTrajectory,
        current_position: Point3,
    ) -> Result<Vec<HitTarget>, TargetSelectionError> {
        if trajectory.predicted.is_empty() {
            return Err(TargetSelectionError::EmptyPrediction);
        }
        // 5 ms 예측 행을 전부 IK로 풀 필요는 없다. 접수 구간을
        // 균등 표본하여 공간적으로 다른 후보만 남긴다.
        const LEVELS: usize = 9;
        let mut candidates: Vec<HitTarget> = Vec::with_capacity(LEVELS);
        for level in 0..LEVELS {
            let fraction = level as f64 / (LEVELS - 1) as f64;
            let y = self.y_min + (self.y_max - self.y_min) * fraction;
            if let Some(candidate) = interpolate_at_y(&trajectory.predicted, y)
                && candidates
                    .iter()
                    .all(|existing| (existing.time_secs - candidate.time_secs).abs() > 1e-9)
            {
                candidates.push(candidate);
            }
        }
        // 예측이 구간 경계까지 못 간 짧은 궤적은 구간 안의
        // 실제 샘플 중 중앙에 가장 가까운 한 점을 사용한다.
        if candidates.is_empty() {
            let target_y = (self.y_min + self.y_max) * 0.5;
            if let Some(sample) = trajectory
                .predicted
                .iter()
                .filter(|sample| sample.position.y >= self.y_min && sample.position.y <= self.y_max)
                .min_by(|left, right| {
                    (left.position.y - target_y)
                        .abs()
                        .total_cmp(&(right.position.y - target_y).abs())
                })
            {
                candidates.push((*sample).into());
            } else {
                return Err(TargetSelectionError::OutsideWindow);
            }
        }
        candidates.sort_by(|left, right| {
            candidate_score(*right, current_position)
                .total_cmp(&candidate_score(*left, current_position))
        });
        return Ok(candidates);
    }
}

fn candidate_score(candidate: HitTarget, current_position: Point3) -> f64 {
    // 단순 위치 차단 단계의 목적 함수: 준비 시간은 늘리고, 이동 거리와 테이블에 너무
    // 가까운 낮은 자세는 줄인다. 최종 도달 가능성은 PositionController가 물리 한계로 판정한다.
    let travel_distance = (candidate.position - current_position).norm();
    return candidate.time_secs - travel_distance / 1.0 + candidate.position.z * 0.05;
}

fn interpolate_at_y(samples: &[TrajectorySample], y: f64) -> Option<HitTarget> {
    for sample in samples {
        if sample.position.y == y {
            return Some((*sample).into());
        }
    }
    for pair in samples.windows(2) {
        let (left, right) = (pair[0], pair[1]);
        let dy = right.position.y - left.position.y;
        if dy.abs() <= f64::EPSILON || (y - left.position.y) * (y - right.position.y) > 0.0 {
            continue;
        }
        let ratio = (y - left.position.y) / dy;
        return Some(HitTarget {
            position: Point3::from(left.position.coords + (right.position - left.position) * ratio),
            incoming_velocity: left.velocity + (right.velocity - left.velocity) * ratio,
            time_secs: left.time_secs + (right.time_secs - left.time_secs) * ratio,
        });
    }
    return None;
}

impl From<TrajectorySample> for HitTarget {
    fn from(sample: TrajectorySample) -> Self {
        return Self {
            position: sample.position,
            incoming_velocity: sample.velocity,
            time_secs: sample.time_secs,
        };
    }
}

/// real과 sim이 함께 쓰는 위치 이동 계획기.
pub struct PositionController;

/// 앞당긴 접수점에서도 타격 시간을 확보하도록 0.10초부터 정밀 예측을 허용한다.
/// 단일 관측으로 확정하지 않고 기존 3회 연속·10cm 수렴 조건은 유지한다.
pub const REFINED_MIN_OBSERVATION_SECS: f64 = 0.10;
pub const REFINED_TARGET_TOLERANCE_M: f64 = 0.10;
const REFINED_STABLE_SAMPLES: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PredictionStage {
    Provisional,
    Refined,
}

/// sim·real이 같은 순서로 실행하는 공 하나의 정렬 단계.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlignmentAction {
    /// 첫 정밀 예측: 중앙 조준 통합 IK로 레일·팔 목표를 확정한다.
    PrimaryRailAndArm,
    /// 확정 후 최신 예측: 확정된 레일에서 팔만 미세 보정한다.
    ArmCorrection,
}

impl AlignmentAction {
    pub fn commands_rail(self) -> bool {
        return matches!(self, Self::PrimaryRailAndArm);
    }
}

/// 트랙 하나에서 본 레일 명령이 한 번만 나가게 한다.
#[derive(Debug, Default, Clone)]
pub struct AlignmentLatch {
    track_seq: Option<u64>,
    primary_sent: bool,
}

impl AlignmentLatch {
    pub fn next_action(&mut self, track_seq: u64, refined_ready: bool) -> Option<AlignmentAction> {
        if self.track_seq != Some(track_seq) {
            *self = Self::default();
            self.track_seq = Some(track_seq);
        }
        if !refined_ready && !self.primary_sent {
            return None;
        }
        return Some(if self.primary_sent {
            AlignmentAction::ArmCorrection
        } else {
            AlignmentAction::PrimaryRailAndArm
        });
    }

    pub fn mark_rail_sent(&mut self, action: AlignmentAction) {
        if matches!(action, AlignmentAction::PrimaryRailAndArm) {
            self.primary_sent = true;
        }
    }

    pub fn mark_primary_sent(&mut self) {
        self.primary_sent = true;
    }

    pub fn track_seq(&self) -> Option<u64> {
        return self.track_seq;
    }

    pub fn primary_sent(&self) -> bool {
        return self.primary_sent;
    }
}

/// 레일 명령을 먼저 내리기 위한 1단계 계획.
///
/// 실물 AXL이 안전 범위로 클램프한 실제 목표를 받은 뒤
/// [`AlignmentController::plan_joints`]로 관절 궤적을 만든다.
#[derive(Debug, Clone)]
pub struct AlignmentPreparation {
    pub action: AlignmentAction,
    pub rail_target_m: Option<f64>,
    primary_joints: Option<Joints>,
}

/// 예측 단계에서 레일 목표와 고정-레일 팔 IK를 만드는 공통 계획기.
pub struct AlignmentController;

impl AlignmentController {
    pub fn prepare(
        arm: &Arm,
        start: &Pose,
        ball: Point3,
        action: AlignmentAction,
    ) -> Result<AlignmentPreparation, DomainError> {
        return match action {
            AlignmentAction::PrimaryRailAndArm => {
                let aligned = Planner::ball_alignment_pose(arm, start, ball)?;
                // 레일 명령 전에 통합해가 정지→정지 궤적으로도
                // 실행 가능한지 검증한다.
                let fixed_start = Pose::new(aligned.rail_x, start.joints.clone());
                Planner::move_to(arm, &fixed_start, aligned.joints.clone(), aligned.rail_x)?;
                Ok(AlignmentPreparation {
                    action,
                    rail_target_m: arm.rail.map(|_| aligned.rail_x),
                    primary_joints: Some(aligned.joints),
                })
            }
            AlignmentAction::ArmCorrection => Ok(AlignmentPreparation {
                action,
                rail_target_m: None,
                primary_joints: None,
            }),
        };
    }

    /// AXL/시뮬이 적용한 레일 위치에서 관절만 이동하는 궤적을 만든다.
    pub fn plan_joints(
        arm: &Arm,
        start: &Pose,
        ball: Point3,
        prepared: &AlignmentPreparation,
        applied_rail_m: Option<f64>,
    ) -> Result<Trajectory, DomainError> {
        let rail_x = applied_rail_m.unwrap_or(start.rail_x);
        let planning_start = Pose::new(rail_x, start.joints.clone());
        if let (Some(primary_joints), Some(requested_rail)) =
            (&prepared.primary_joints, prepared.rail_target_m)
            && (requested_rail - rail_x).abs() <= 1e-6
        {
            return Planner::move_to(arm, &planning_start, primary_joints.clone(), rail_x);
        }
        // 하드웨어 클램프로 요청 레일과 적용 레일이 다르거나
        // 후속 보정이면 실제 레일에서 자세 IK를 다시 푼다.
        return Planner::ball_alignment_fixed_rail(arm, &planning_start, ball);
    }
}

/// 전역 IK가 다른 팔 접힘 가지를 골라 듀얼 베이스가 튀는 것을 막는 한계.
/// 낮아진 레일 베이스에서 한쪽 끝 접수점까지 전개할 때 실측 기구학상 약 56°가
/// 필요하므로 60°까지 허용한다.
/// 속도·가속도 한계는 궤적 계획기가 별도로 검사한다.
pub const MAX_ALIGNMENT_BASE_STEP_RAD: f64 = 60.0_f64.to_radians();

pub fn alignment_base_step_rad(start: &Pose, trajectory: &Trajectory) -> f64 {
    return trajectory
        .follow_through
        .values
        .first()
        .zip(start.joints.values.first())
        .map_or(0.0, |(target, measured)| target - measured);
}

/// 물리 한계 최단시간 대비 여유. sim·real 모두 같은 값을 쓴다.
pub const RAIL_MOVE_DURATION_SCALE: f64 = 1.25;

pub fn alignment_rail_move_duration(arm: &Arm, start_x: f64, target_x: f64) -> f64 {
    let Some(rail) = arm.rail else {
        return crate::defaults::RETURN_TO_CENTER_MIN_SECS;
    };
    let distance = (target_x - start_x).abs();
    if distance <= f64::EPSILON {
        return crate::defaults::RETURN_TO_CENTER_MIN_SECS;
    }
    let acceleration = crate::defaults::RAIL_ACCEL_M_S2;
    let max_speed = rail.max_speed;
    let ramp_distance = max_speed * max_speed / acceleration;
    let minimum = if distance <= ramp_distance {
        2.0 * (distance / acceleration).sqrt()
    } else {
        distance / max_speed + max_speed / acceleration
    };
    return (minimum * RAIL_MOVE_DURATION_SCALE).max(0.02);
}

/// 실기와 시뮬레이션이 함께 사용하는 라켓 수평 조준축.
/// 4-DOF 논리 관절 1번은 Dynamixel ID 3에 대응한다.
pub const DIRECT_AIM_JOINT_INDEX: usize = 1;
pub const MIN_DIRECT_COMMAND_SECS: f64 = 0.05;
pub const MAX_DIRECT_COMMAND_SECS: f64 = 0.30;

/// 공 예측 한 건에서 계산된 레일·라켓 조준 직접 명령.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectControlCommand {
    pub stage: PredictionStage,
    pub target: HitTarget,
    pub rail_x: f64,
    pub aim_rad: f64,
    pub duration_secs: f64,
}

/// 명령 뒤 읽은 실제 위치와의 차이(`commanded - measured`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectControlMeasurement {
    pub rail_commanded_m: f64,
    pub rail_measured_m: f64,
    pub rail_error_m: f64,
    pub aim_commanded_rad: f64,
    pub aim_measured_rad: f64,
    pub aim_error_rad: f64,
}

impl DirectControlCommand {
    pub fn compare_with_pose(&self, pose: &Pose) -> Option<DirectControlMeasurement> {
        return DirectControlMeasurement::from_commanded(self.rail_x, self.aim_rad, pose);
    }
}

impl DirectControlMeasurement {
    pub fn from_commanded(
        rail_commanded_m: f64,
        aim_commanded_rad: f64,
        pose: &Pose,
    ) -> Option<Self> {
        let aim_measured_rad = *pose.joints.values.get(DIRECT_AIM_JOINT_INDEX)?;
        return Some(Self {
            rail_commanded_m,
            rail_measured_m: pose.rail_x,
            rail_error_m: rail_commanded_m - pose.rail_x,
            aim_commanded_rad,
            aim_measured_rad,
            aim_error_rad: aim_commanded_rad - aim_measured_rad,
        });
    }
}

/// 전체 팔 IK 대신 라켓 헤드 x를 공 x에 맞추고, 레일 위치를
/// 단일 각도 함수에 넣어 상대편 끝선 중앙을 바라보게 하는 보존 제어기.
/// GUI sim의 목표 선택과 진단에 남아 있으며, 활성 real 워커는 전체 자세 IK를
/// 푸는 [`crate::robot::motion::Planner::ball_alignment`]을 사용한다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectController {
    selector: HitTargetSelector,
}

impl DirectController {
    pub fn new(y_min: f64, y_max: f64) -> Result<Self, DirectControlError> {
        let selector = HitTargetSelector::new(y_min, y_max).map_err(DirectControlError::Target)?;
        return Ok(Self { selector });
    }

    pub fn select_target(
        &self,
        trajectory: &BallTrajectory,
    ) -> Result<HitTarget, DirectControlError> {
        return self
            .selector
            .select(trajectory)
            .map_err(DirectControlError::Target);
    }

    /// `elapsed_secs`는 예측 기준 시각부터 명령 계산까지 흐른 시간이다.
    pub fn command(
        &self,
        arm: &Arm,
        start: &Pose,
        trajectory: &BallTrajectory,
        stage: PredictionStage,
        elapsed_secs: f64,
    ) -> Result<DirectControlCommand, DirectControlError> {
        if !elapsed_secs.is_finite() || elapsed_secs < 0.0 {
            return Err(DirectControlError::InvalidElapsed);
        }
        let target = self.select_target(trajectory)?;
        return self.command_for_target(arm, start, target, stage, elapsed_secs);
    }

    pub fn command_for_target(
        &self,
        arm: &Arm,
        start: &Pose,
        target: HitTarget,
        stage: PredictionStage,
        elapsed_secs: f64,
    ) -> Result<DirectControlCommand, DirectControlError> {
        if !elapsed_secs.is_finite() || elapsed_secs < 0.0 {
            return Err(DirectControlError::InvalidElapsed);
        }
        let remaining_secs = target.time_secs - elapsed_secs;
        if !remaining_secs.is_finite() || remaining_secs <= 0.0 {
            return Err(DirectControlError::Expired {
                late_by_secs: -remaining_secs,
            });
        }
        let rail_x = rail_for_racket_head_x(arm, start, target.position.x)?;
        let aim_rad = aim_angle_for_rail(arm, rail_x)?;
        let rail_distance_m = (rail_x - start.rail_x).abs();
        let rail_required_secs = arm.rail.map_or(0.0, |rail| {
            minimum_trapezoid_time(
                rail_distance_m,
                rail.max_speed,
                crate::defaults::rail::RAIL_ACCEL_M_S2,
            )
        });
        let aim_current_rad = *start
            .joints
            .values
            .get(DIRECT_AIM_JOINT_INDEX)
            .ok_or(DirectControlError::MissingAimJoint)?;
        let aim_required_secs = (aim_rad - aim_current_rad).abs() / arm.max_joint_speed.max(1e-9);
        let required_secs = rail_required_secs.max(aim_required_secs);
        if required_secs > remaining_secs {
            return Err(DirectControlError::InsufficientTime {
                remaining_secs,
                required_secs,
            });
        }
        return Ok(DirectControlCommand {
            stage,
            target,
            rail_x,
            aim_rad,
            duration_secs: remaining_secs
                .min(MAX_DIRECT_COMMAND_SECS)
                .max(MIN_DIRECT_COMMAND_SECS.min(remaining_secs))
                .max(required_secs),
        });
    }
}

/// 레일 위치에서 상대편 탁구대 끝선 중앙을 바라보는 수평 조준각.
pub fn aim_angle_for_rail(arm: &Arm, rail_x: f64) -> Result<f64, DirectControlError> {
    if !rail_x.is_finite() {
        return Err(DirectControlError::InvalidRailPosition);
    }
    let mount_y = arm.rail.map_or(arm.base.y, |rail| rail.mount_y);
    let mount_x = arm.rail.map_or(rail_x, |rail| rail.world_x(rail_x));
    let dx = crate::constants::table::WIDTH_X * 0.5 - mount_x;
    let dy = crate::constants::table::LENGTH_Y - mount_y;
    let requested = dx.atan2(dy);
    return Ok(arm
        .joint_limit(DIRECT_AIM_JOINT_INDEX)
        .map_or(requested, |limit| requested.clamp(limit.min, limit.max)));
}

/// 라켓 헤드의 실제 x가 공 x와 같아지도록 레일을 보정한다.
/// 전체 IK는 쓰지 않고 `aim_angle_for_rail`과 FK 평행이동만 반복한다.
fn rail_for_racket_head_x(arm: &Arm, start: &Pose, ball_x: f64) -> Result<f64, DirectControlError> {
    if !ball_x.is_finite() {
        return Err(DirectControlError::InvalidRailPosition);
    }
    let mut rail_x = arm
        .rail
        .map_or(ball_x, |rail| rail.rail_x_for_world_x(ball_x));
    for _ in 0..4 {
        let mut joints = start.joints.clone();
        let aim = joints
            .values
            .get_mut(DIRECT_AIM_JOINT_INDEX)
            .ok_or(DirectControlError::MissingAimJoint)?;
        *aim = aim_angle_for_rail(arm, rail_x)?;
        let head = arm
            .forward_kinematics_with_rail(rail_x, &joints)
            .ok_or(DirectControlError::ForwardKinematics)?;
        rail_x += ball_x - head.position.x;
        if let Some(rail) = arm.rail {
            rail_x = rail.clamp_x(rail_x);
        }
    }
    return Ok(rail_x);
}

fn minimum_trapezoid_time(distance: f64, max_speed: f64, acceleration: f64) -> f64 {
    if distance <= 0.0 {
        return 0.0;
    }
    let ramp_distance = max_speed * max_speed / acceleration;
    if distance <= ramp_distance {
        return 2.0 * (distance / acceleration).sqrt();
    }
    return 2.0 * max_speed / acceleration + (distance - ramp_distance) / max_speed;
}

/// 한 프레임만 우연히 맞은 예측을 정밀 단계로 착각하지 않도록
/// 최근 목표를 누적한다.
#[derive(Debug, Default)]
pub struct PredictionStability {
    recent_targets: std::collections::VecDeque<Point3>,
    refined: bool,
}

impl PredictionStability {
    pub fn reset(&mut self) {
        self.recent_targets.clear();
        self.refined = false;
    }

    pub fn observe(&mut self, target: Point3, observed_span_secs: f64) -> PredictionStage {
        self.recent_targets.push_back(target);
        while self.recent_targets.len() > REFINED_STABLE_SAMPLES {
            self.recent_targets.pop_front();
        }
        let stable = self.recent_targets.len() == REFINED_STABLE_SAMPLES
            && self
                .recent_targets
                .iter()
                .all(|sample| (*sample - target).norm() <= REFINED_TARGET_TOLERANCE_M);
        self.refined |= observed_span_secs >= REFINED_MIN_OBSERVATION_SECS && stable;
        return if self.refined {
            PredictionStage::Refined
        } else {
            PredictionStage::Provisional
        };
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PositionPlan {
    pub target: HitTarget,
    pub trajectory: Trajectory,
}

impl PositionController {
    /// 점수순 후보를 실제 IK·충돌·속도·가속도·토크·도착시각 검사에 통과시켜
    /// 실행 가능한 첫 후보를 최적 위치로 확정한다.
    pub fn plan_best(
        arm: &Arm,
        start: &Pose,
        ball_trajectory: &BallTrajectory,
        selector: &HitTargetSelector,
    ) -> Result<PositionPlan, PositionControlError> {
        let current = if arm.rail.is_some() {
            arm.forward_kinematics_with_rail(start.rail_x, &start.joints)
        } else {
            arm.forward_kinematics(&start.joints)
        }
        .ok_or_else(|| PositionControlError::Unreachable("현재 라켓 FK 실패".into()))?
        .position;
        let candidates = selector
            .ranked_candidates(ball_trajectory, current)
            .map_err(|error| PositionControlError::Unreachable(error.to_string()))?;
        let elapsed = ball_trajectory.reference_time.elapsed().as_secs_f64();
        let mut last_error = None;
        for hit_target in candidates {
            match Self::plan(arm, start, hit_target.target(), elapsed) {
                Ok(trajectory) => {
                    // `ranked_candidates`는 준비 시간·이동 거리·높이 순으로 이미
                    // 정렬되어 있다. 실행 가능한 첫 후보가 곧 최적 후보이므로
                    // 즉시 반환한다. 예측 샘플 수십 개에 대해 전체 IK·충돌
                    // 계획을 모두 돌리면 실기 반응이 수백 ms~수 초 늦어진다.
                    return Ok(PositionPlan {
                        target: hit_target,
                        trajectory,
                    });
                }
                Err(error) => last_error = Some(error),
            }
        }
        return Err(last_error.unwrap_or_else(|| {
            PositionControlError::Unreachable("실행 가능한 궤적 후보가 없음".into())
        }));
    }

    /// 현재 포즈에서 목표 위치까지 정지→정지 궤적을 만든다.
    ///
    /// `elapsed_secs`는 궤적 기준 시각 이후 이미 흐른 시간이다. 최소 안전 이동 시간이
    /// 남은 시간보다 길면 명령을 만들지 않는다. 일찍 도착한 경우 하드웨어 position
    /// hold가 마지막 자세를 유지한다.
    pub fn plan(
        arm: &Arm,
        start: &Pose,
        target: Target,
        elapsed_secs: f64,
    ) -> Result<Trajectory, PositionControlError> {
        if !target.position.coords.iter().all(|value| value.is_finite())
            || !target.arrival_time_secs.is_finite()
            || !elapsed_secs.is_finite()
        {
            return Err(PositionControlError::InvalidTarget);
        }
        let remaining_secs = target.arrival_time_secs - elapsed_secs;
        if remaining_secs <= 0.0 {
            return Err(PositionControlError::Stale {
                late_by_secs: -remaining_secs,
            });
        }

        // 이 제어기의 계약은 '라켓 중심을 공 위치에 대기'다. 법선까지
        // 맞추는 5차원 자세 IK는 스윙용이며, 위치 제어에서는 필요 없는
        // 실패 분기와 수치 탐색 비용만 만든다.
        let goal = position_only_goal(arm, start, target.position)?;
        let mut trajectory = Planner::move_to(arm, start, goal.joints, goal.rail_x)
            .map_err(|error| PositionControlError::Unreachable(error.to_string()))?;
        if trajectory.duration_secs > remaining_secs {
            return Err(PositionControlError::InsufficientTime {
                remaining_secs,
                required_secs: trajectory.duration_secs,
            });
        }
        // 최소 이동 시간에 먼저 도착한 뒤, 공이 도착할 때까지 종료
        // 자세(속도 0)를 유지한다. 이 hold가 끝나야 복귀 궤적이 시작된다.
        trajectory.duration_secs = remaining_secs;
        return Ok(trajectory);
    }
}

fn position_only_goal(
    arm: &Arm,
    start: &Pose,
    target: Point3,
) -> Result<Pose, PositionControlError> {
    let Some(rail) = arm.rail else {
        let joints = arm
            .inverse_kinematics_near(target, Some(&start.joints))
            .map_err(|error| PositionControlError::Unreachable(error.to_string()))?;
        return Ok(Pose::new(start.rail_x, joints));
    };

    // 현재 레일, 표적 x 직하, 그 중간을 풀어 레일·관절 동시 이동
    // 시간이 짧은 해를 고른다. 모두 고정 레일의 3차원 위치 IK이다.
    let target_rail = rail.rail_x_for_world_x(target.x);
    let rail_candidates = [
        rail.clamp_x(start.rail_x),
        target_rail,
        rail.clamp_x((start.rail_x + target_rail) * 0.5),
    ];
    let mut best: Option<(f64, Pose)> = None;
    let mut last_error = None;
    let mut attempted_rails: Vec<f64> = Vec::with_capacity(rail_candidates.len());
    for rail_x in rail_candidates {
        if attempted_rails
            .iter()
            .any(|attempted| (*attempted - rail_x).abs() <= 1e-9)
        {
            continue;
        }
        attempted_rails.push(rail_x);
        match arm.inverse_kinematics_with_rail(&rail, rail_x, target, Some(&start.joints)) {
            Ok(joints) => {
                let rail_secs = (rail_x - start.rail_x).abs() / rail.max_speed.max(1e-9);
                let joint_secs = joints
                    .values
                    .iter()
                    .zip(&start.joints.values)
                    .map(|(goal, current)| (goal - current).abs() / arm.max_joint_speed.max(1e-9))
                    .fold(0.0_f64, f64::max);
                let score = rail_secs.max(joint_secs);
                if best
                    .as_ref()
                    .is_none_or(|(best_score, _)| score < *best_score)
                {
                    best = Some((score, Pose::new(rail_x, joints)));
                }
            }
            Err(error) => last_error = Some(error),
        }
    }
    if let Some((_, pose)) = best {
        return Ok(pose);
    }
    return Err(PositionControlError::Unreachable(
        last_error
            .map(|error| error.to_string())
            .unwrap_or_else(|| "위치 IK 해 없음".into()),
    ));
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum TargetSelectionError {
    #[error("목표 선택 y 구간이 잘못됨")]
    InvalidWindow,
    #[error("미래 예측 궤적이 비어 있음")]
    EmptyPrediction,
    #[error("미래 궤적이 라켓 작업 구간을 지나지 않음")]
    OutsideWindow,
}

#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum DirectControlError {
    #[error("예측 기준 경과 시간이 유효하지 않음")]
    InvalidElapsed,
    #[error("목표 시각이 {late_by_secs:.3}s 지남")]
    Expired { late_by_secs: f64 },
    #[error("목표 선택 실패: {0}")]
    Target(TargetSelectionError),
    #[error("레일 또는 공 x 위치가 유효하지 않음")]
    InvalidRailPosition,
    #[error("현재 포즈에 라켓 수평 조준축이 없음")]
    MissingAimJoint,
    #[error("라켓 헤드 위치 계산 실패")]
    ForwardKinematics,
    #[error("남은 시간 {remaining_secs:.3}s, 레일·조준축에 필요한 최소 시간 {required_secs:.3}s")]
    InsufficientTime {
        remaining_secs: f64,
        required_secs: f64,
    },
}

#[derive(Debug, Clone, PartialEq, Error)]
pub enum PositionControlError {
    #[error("목표 위치 또는 도착 시간이 유효하지 않음")]
    InvalidTarget,
    #[error("목표 시각이 {late_by_secs:.3}s 지남")]
    Stale { late_by_secs: f64 },
    #[error("목표에 도달할 수 없음: {0}")]
    Unreachable(String),
    #[error("남은 시간 {remaining_secs:.3}s, 필요 시간 {required_secs:.3}s")]
    InsufficientTime {
        remaining_secs: f64,
        required_secs: f64,
    },
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    #[test]
    fn alignment_latch_waits_for_refined_and_commands_rail_once() {
        let mut latch = AlignmentLatch::default();

        assert_eq!(latch.next_action(7, false), None);
        assert_eq!(
            latch.next_action(7, true),
            Some(AlignmentAction::PrimaryRailAndArm)
        );
        latch.mark_rail_sent(AlignmentAction::PrimaryRailAndArm);
        assert_eq!(
            latch.next_action(7, true),
            Some(AlignmentAction::ArmCorrection)
        );

        assert_eq!(
            latch.next_action(8, false),
            None,
            "새 공 트랙은 본 예측이 안정될 때까지 명령하지 않아야 함"
        );
    }

    #[test]
    fn selector_interpolates_one_n_by_seven_segment() {
        let trajectory = BallTrajectory::new(
            vec![],
            vec![
                TrajectorySample::new(
                    Point3::new(0.2, 0.5, 0.4),
                    Vector3::new(1.0, -2.0, 0.0),
                    0.1,
                ),
                TrajectorySample::new(
                    Point3::new(0.4, 0.1, 0.2),
                    Vector3::new(3.0, -4.0, -2.0),
                    0.3,
                ),
            ],
            Instant::now(),
        )
        .unwrap();
        let selected = HitTargetSelector::new(0.2, 0.4)
            .unwrap()
            .select(&trajectory)
            .unwrap();
        assert!((selected.position.x - 0.3).abs() < 1e-12);
        assert!((selected.position.y - 0.3).abs() < 1e-12);
        assert!((selected.incoming_velocity.x - 2.0).abs() < 1e-12);
        assert!((selected.time_secs - 0.2).abs() < 1e-12);
    }

    #[test]
    fn direct_controller_aligns_racket_head_and_aims_at_opponent_end() {
        let robot = crate::defaults::robot().unwrap();
        let trajectory = BallTrajectory::new(
            vec![],
            vec![
                TrajectorySample::new(Point3::new(0.2, 0.5, 0.4), Vector3::zeros(), 0.8),
                TrajectorySample::new(Point3::new(0.4, 0.1, 0.2), Vector3::zeros(), 1.0),
            ],
            Instant::now(),
        )
        .unwrap();
        let controller = DirectController::new(0.2, 0.4).unwrap();
        let start = Pose::new(
            robot.arm.rail.as_ref().unwrap().default_x(),
            robot.arm.default_joints.clone(),
        );

        let provisional = controller
            .command(
                &robot.arm,
                &start,
                &trajectory,
                PredictionStage::Provisional,
                0.0,
            )
            .unwrap();
        let refined = controller
            .command(
                &robot.arm,
                &start,
                &trajectory,
                PredictionStage::Refined,
                0.0,
            )
            .unwrap();

        assert!((provisional.rail_x - refined.rail_x).abs() < 1e-12);
        assert!((provisional.aim_rad - refined.aim_rad).abs() < 1e-12);
        let mut commanded = start.joints.clone();
        commanded.values[DIRECT_AIM_JOINT_INDEX] = refined.aim_rad;
        let racket = robot
            .arm
            .forward_kinematics_with_rail(refined.rail_x, &commanded)
            .unwrap();
        assert!((racket.position.x - refined.target.position.x).abs() < 1e-5);
        let toward_far_center = Vector3::new(
            crate::constants::table::WIDTH_X * 0.5 - racket.position.x,
            crate::constants::table::LENGTH_Y - racket.position.y,
            0.0,
        )
        .normalize();
        let racket_facing_xy = Vector3::new(racket.normal.x, racket.normal.y, 0.0).normalize();
        assert!(
            racket_facing_xy.dot(&toward_far_center) > 0.99,
            "라켓 수평 법선이 상대편 끝선 중앙을 향해야 함"
        );
        assert!(
            refined.aim_rad > 0.0,
            "왼쪽 공에서는 오른쪽 중앙을 향해야 함"
        );
    }

    #[test]
    fn direct_measurement_is_commanded_minus_measured() {
        let command = DirectControlCommand {
            stage: PredictionStage::Refined,
            target: HitTarget {
                position: Point3::new(0.3, 0.3, 0.2),
                incoming_velocity: Vector3::zeros(),
                time_secs: 0.2,
            },
            rail_x: 0.30,
            aim_rad: -0.10,
            duration_secs: 0.1,
        };
        let pose = Pose::new(
            0.27,
            super::super::Joints::from_slice(&[0.0, -0.15, 0.0, -0.45]),
        );
        let measured = command.compare_with_pose(&pose).unwrap();

        assert!((measured.rail_error_m - 0.03).abs() < 1e-12);
        assert!((measured.aim_error_rad - 0.05).abs() < 1e-12);
    }

    #[test]
    fn direct_controller_rejects_a_rail_target_that_cannot_arrive_in_time() {
        let robot = crate::defaults::robot().unwrap();
        let controller = DirectController::new(0.2, 0.4).unwrap();
        let start = Pose::new(0.0, robot.arm.default_joints.clone());
        let target = HitTarget {
            position: Point3::new(1.0, 0.3, 0.2),
            incoming_velocity: Vector3::zeros(),
            time_secs: 0.05,
        };

        let error = controller
            .command_for_target(
                &robot.arm,
                &start,
                target,
                PredictionStage::Provisional,
                0.0,
            )
            .unwrap_err();

        assert!(matches!(error, DirectControlError::InsufficientTime { .. }));
    }

    #[test]
    fn direct_controller_does_not_reserve_fixed_impact_minimum() {
        let robot = crate::defaults::robot().unwrap();
        let controller = DirectController::new(0.2, 0.4).unwrap();
        let rail_x = robot.arm.rail.expect("rail").default_x();
        let start = Pose::new(rail_x, robot.arm.default_joints.clone());
        let racket = robot
            .arm
            .forward_kinematics_with_rail(start.rail_x, &start.joints)
            .expect("FK");
        let target = HitTarget {
            position: Point3::new(racket.position.x, 0.3, racket.position.z),
            incoming_velocity: Vector3::zeros(),
            time_secs: 0.20,
        };

        let command = controller
            .command_for_target(
                &robot.arm,
                &start,
                target,
                PredictionStage::Provisional,
                0.0,
            )
            .expect("0.25초보다 짧아도 레일·조준이 가능하면 허용");

        assert!(command.target.time_secs < crate::defaults::motion::FIXED_IMPACT_MIN_DURATION_SECS);
    }

    #[test]
    fn refined_stage_needs_time_and_three_targets_within_ten_centimeters() {
        let mut stability = PredictionStability::default();
        assert_eq!(
            stability.observe(Point3::new(0.0, 0.3, 0.4), 0.05),
            PredictionStage::Provisional
        );
        assert_eq!(
            stability.observe(Point3::new(0.04, 0.3, 0.4), 0.09),
            PredictionStage::Provisional
        );
        assert_eq!(
            stability.observe(Point3::new(0.05, 0.3, 0.4), 0.10),
            PredictionStage::Refined
        );
        // 정밀 단계는 한 번 성립하면 다시 1차로 내려가지 않는다.
        assert_eq!(
            stability.observe(Point3::new(0.30, 0.3, 0.4), 0.30),
            PredictionStage::Refined
        );
    }

    #[test]
    fn refined_stage_rejects_a_single_large_prediction_jump() {
        let mut stability = PredictionStability::default();
        stability.observe(Point3::new(0.0, 0.3, 0.4), 0.10);
        stability.observe(Point3::new(0.02, 0.3, 0.4), 0.11);
        assert_eq!(
            stability.observe(Point3::new(0.20, 0.3, 0.4), 0.12),
            PredictionStage::Provisional
        );
    }

    #[test]
    fn stale_target_never_creates_motion() {
        let robot = crate::defaults::robot().unwrap();
        let rail_x = robot.arm.rail.as_ref().unwrap().default_x();
        let start = Pose::new(rail_x, robot.arm.default_joints.clone());
        let position = robot
            .arm
            .forward_kinematics_with_rail(rail_x, &start.joints)
            .unwrap()
            .position;
        let error = PositionController::plan(
            &robot.arm,
            &start,
            Target {
                position,
                arrival_time_secs: 0.1,
            },
            0.2,
        )
        .unwrap_err();
        assert!(matches!(error, PositionControlError::Stale { .. }));
    }

    #[test]
    fn unreachable_position_is_rejected_without_motion() {
        let robot = crate::defaults::robot().unwrap();
        let rail_x = robot.arm.rail.as_ref().unwrap().default_x();
        let start = Pose::new(rail_x, robot.arm.default_joints.clone());
        let error = PositionController::plan(
            &robot.arm,
            &start,
            Target {
                position: Point3::new(10.0, 10.0, 10.0),
                arrival_time_secs: 10.0,
            },
            0.0,
        )
        .unwrap_err();
        assert!(matches!(error, PositionControlError::Unreachable(_)));
    }

    #[test]
    fn center_move_respects_the_real_rail_acceleration() {
        let robot = crate::defaults::robot().unwrap();
        let rail = robot.arm.rail.expect("기본 레일");
        let start = Pose::new(rail.x_max, robot.arm.default_joints.clone());
        let trajectory = Planner::return_to_center(&robot.arm, &start).unwrap();
        assert!(
            trajectory.peak_rail_acceleration() <= crate::defaults::rail::RAIL_ACCEL_M_S2 + 1e-9
        );
        assert!(
            trajectory.duration_secs > 0.36,
            "0.702 m를 0.36 s로 이동하던 실기 회귀를 막아야 함"
        );
    }

    #[test]
    fn default_sim_shot_has_a_reachable_position_target() {
        let robot = crate::defaults::robot().unwrap();
        let rail_x = robot.arm.rail.as_ref().unwrap().default_x();
        let start = Pose::new(rail_x, robot.arm.default_joints.clone());
        let settings = crate::sim::launch::Settings::default();
        let position = settings.muzzle_position();
        let velocity = settings.launch_velocity();
        let omega = settings.launch_angular_velocity();
        let predicted = crate::estimator::Kinematics::sample_trajectory(
            Vector3::new(position.x.into(), position.y.into(), position.z.into()),
            Vector3::new(velocity.x.into(), velocity.y.into(), velocity.z.into()),
            Vector3::new(omega.x.into(), omega.y.into(), omega.z.into()),
            &crate::defaults::PhysicsParams::default(),
        );
        let trajectory = BallTrajectory::new(vec![], predicted, Instant::now()).unwrap();
        let window = crate::robot::motion::InterceptWindow::default();
        let selector = HitTargetSelector::new(window.y_min, window.y_max).unwrap();
        let launch_position = Point3::new(position.x.into(), position.y.into(), position.z.into());
        let candidates = selector
            .ranked_candidates(&trajectory, launch_position)
            .unwrap();
        assert!(candidates.len() <= 9, "IK 후보는 균등 표본 9개 이하");
        let planned = PositionController::plan_best(&robot.arm, &start, &trajectory, &selector)
            .expect("기본 sim 샷의 위치 목표는 실행 가능해야 함");
        let reached = robot
            .arm
            .forward_kinematics_with_rail(
                planned.trajectory.follow_through_rail_x,
                &planned.trajectory.end_joints(),
            )
            .expect("계획 종료 자세 FK");
        assert!(
            (reached.position - planned.target.position).norm()
                <= crate::robot::Arm::POSE_IK_POSITION_TOLERANCE
        );
    }
}
