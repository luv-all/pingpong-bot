//! 공 궤적에서 목표를 고르고 라켓을 그 위치까지 옮기는 공통 제어 경계.

use nalgebra::Vector3;
use rayon::prelude::*;
use thiserror::Error;

use crate::Point3;
use crate::estimator::{BallTrajectory, TrajectorySample};

use super::motion::{Planner, Trajectory};
use super::{Arm, Pose};

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

/// 위치 제어용 타격 후보 Y 간격. 예측 적분의 5ms 표본을
/// 전부 IK로 풀면 실시간 제어가 느려지므로, 공간적으로 다른 점만
/// 남긴다. 각 점은 [`PositionController::plan_best`]에서 레일 포함
/// 역기구학과 전체 궤적 제약을 실제로 통과해야 채택된다.
const TARGET_CANDIDATE_SPACING_M: f64 = 0.025;

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
        // 균등 표본하여 공간적으로 다른 후보만 남긴다. 고정 9개를
        // 쓰면 창을 넓혔을 때 간격도 함께 벌어지므로 2.5cm 간격을 유지한다.
        let intervals = ((self.y_max - self.y_min) / TARGET_CANDIDATE_SPACING_M)
            .ceil()
            .max(1.0) as usize;
        let levels = intervals + 1;
        let mut candidates: Vec<HitTarget> = Vec::with_capacity(levels);
        for level in 0..levels {
            let fraction = level as f64 / intervals as f64;
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

/// 제어 반응성 확인을 위해 150 ms를 관측하고, 최근 예측 3개의
/// 수렴폭 10 cm를 함께 만족하면 정밀 예측으로 올린다.
pub const REFINED_MIN_OBSERVATION_SECS: f64 = 0.15;
pub const REFINED_TARGET_TOLERANCE_M: f64 = 0.10;
const REFINED_STABLE_SAMPLES: usize = 3;
/// 1차는 빠른 반응을 위해 50 ms·2개 표본만 사용하되,
/// 서로 30 cm를 넘게 튀는 초기 예측은 실물에 보내지 않는다.
pub const PROVISIONAL_MIN_OBSERVATION_SECS: f64 = 0.05;
pub const PROVISIONAL_TARGET_TOLERANCE_M: f64 = 0.30;
const PROVISIONAL_STABLE_SAMPLES: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PredictionStage {
    Provisional,
    Refined,
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

    /// 최근 2개 예측이 30 cm 안에 모였고 최소 50 ms를 관측했는지.
    /// 즉시 예측을 실물에 보내지 않고, 이 조건을 1차 명령 게이트로 쓴다.
    pub fn provisional_ready(&self, observed_span_secs: f64) -> bool {
        let Some(latest) = self.recent_targets.back() else {
            return false;
        };
        return observed_span_secs >= PROVISIONAL_MIN_OBSERVATION_SECS
            && self.recent_targets.len() >= PROVISIONAL_STABLE_SAMPLES
            && self
                .recent_targets
                .iter()
                .rev()
                .take(PROVISIONAL_STABLE_SAMPLES)
                .all(|sample| (*sample - *latest).norm() <= PROVISIONAL_TARGET_TOLERANCE_M);
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PositionPlan {
    pub target: HitTarget,
    pub trajectory: Trajectory,
    /// 공 도착 시각 안에 완료되는 정식 계획인지. false면 시뮬 가시성용
    /// best-effort 이동이라 공을 놓칠 수 있지만 로봇은 안전 한계 안에서 움직인다.
    pub arrives_on_time: bool,
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
        // 후보 순서는 점수순으로 보존하면서 계산만 Rayon으로 병렬화한다.
        // `collect`가 IndexedParallelIterator의 원래 순서를 유지하므로 가장 앞의
        // 성공 후보를 고르는 기존 의미는 바뀌지 않는다. Rayon 기본 풀은 머신의
        // 가용 논리 CPU 수를 사용한다.
        let attempts: Vec<_> = candidates
            .par_iter()
            .copied()
            .map(|hit_target| {
                (
                    hit_target,
                    Self::plan(arm, start, hit_target.target(), elapsed),
                )
            })
            .collect();
        for (hit_target, result) in &attempts {
            if let Ok(trajectory) = result {
                return Ok(PositionPlan {
                    target: *hit_target,
                    trajectory: trajectory.clone(),
                    arrives_on_time: true,
                });
            }
        }
        // 마지막 후보의 실패를 그대로 보여주면, 공이 이미 거의 지나간
        // 어떤 평면의 `0.005 s`가 전체 계획의 남은 시간처럼 보인다. 시간 부족
        // 후보 중에서 실제 성공 가능성(남은/필요 시간)이 가장 높았던 것을
        // 보고해야 진단값이 선택 문제를 대표한다.
        let best_timing_error = attempts
            .iter()
            .filter_map(|(_, result)| match result {
                Err(PositionControlError::InsufficientTime {
                    remaining_secs,
                    required_secs,
                }) => Some((remaining_secs / required_secs.max(f64::EPSILON), result)),
                _ => None,
            })
            .max_by(|(left, _), (right, _)| left.total_cmp(right))
            .and_then(|(_, result)| result.clone().err());
        return Err(best_timing_error.unwrap_or_else(|| {
            attempts
                .into_iter()
                .find_map(|(_, result)| result.err())
                .unwrap_or_else(|| {
                    PositionControlError::Unreachable("실행 가능한 궤적 후보가 없음".into())
                })
        }));
    }

    /// 정시 도달 계획이 없으면 점수순 첫 도달 가능 후보로 안전하게 이동한다.
    ///
    /// sim에서 "계획 실패 = 완전 정지"로 보이는 문제를 진단하기 위한 경로다.
    /// 실기 [`crate::pipeline`]는 계속 [`Self::plan_best`]만 사용하므로 늦은 명령을
    /// 하드웨어에 보내지 않는다.
    pub fn plan_best_or_reachable(
        arm: &Arm,
        start: &Pose,
        ball_trajectory: &BallTrajectory,
        selector: &HitTargetSelector,
    ) -> Result<PositionPlan, PositionControlError> {
        match Self::plan_best(arm, start, ball_trajectory, selector) {
            Ok(planned) => return Ok(planned),
            Err(PositionControlError::InvalidTarget) => {
                return Err(PositionControlError::InvalidTarget);
            }
            Err(_) => {}
        }

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
        let attempts: Vec<_> = candidates
            .par_iter()
            .copied()
            .map(|hit_target| {
                (
                    hit_target,
                    Self::plan_reachable(arm, start, hit_target.position),
                )
            })
            .collect();
        for (hit_target, result) in &attempts {
            if let Ok(trajectory) = result {
                return Ok(PositionPlan {
                    target: *hit_target,
                    trajectory: trajectory.clone(),
                    arrives_on_time: false,
                });
            }
        }
        return Err(attempts
            .into_iter()
            .rev()
            .find_map(|(_, result)| result.err())
            .unwrap_or_else(|| {
                PositionControlError::Unreachable("도달 가능한 IK 후보 없음".into())
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

    /// 도착 시각은 무시하되 IK·관절/레일 속도·가속도·토크 한계는 지키는 이동.
    fn plan_reachable(
        arm: &Arm,
        start: &Pose,
        target: Point3,
    ) -> Result<Trajectory, PositionControlError> {
        if !target.coords.iter().all(|value| value.is_finite()) {
            return Err(PositionControlError::InvalidTarget);
        }
        let goal = position_only_goal(arm, start, target)?;
        return Planner::move_to(arm, start, goal.joints, goal.rail_x)
            .map_err(|error| PositionControlError::Unreachable(error.to_string()));
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
    let target_rail = rail.clamp_x(target.x);
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
    fn refined_stage_needs_150ms_and_three_targets_within_ten_centimeters() {
        let mut stability = PredictionStability::default();
        assert_eq!(
            stability.observe(Point3::new(0.0, 0.3, 0.4), 0.10),
            PredictionStage::Provisional
        );
        assert_eq!(
            stability.observe(Point3::new(0.04, 0.3, 0.4), 0.14),
            PredictionStage::Provisional
        );
        assert_eq!(
            stability.observe(Point3::new(0.05, 0.3, 0.4), 0.15),
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
        stability.observe(Point3::new(0.0, 0.3, 0.4), 0.25);
        stability.observe(Point3::new(0.02, 0.3, 0.4), 0.26);
        assert_eq!(
            stability.observe(Point3::new(0.20, 0.3, 0.4), 0.27),
            PredictionStage::Provisional
        );
    }

    #[test]
    fn provisional_command_waits_for_two_reasonably_stable_targets_and_fifty_ms() {
        let mut stability = PredictionStability::default();
        stability.observe(Point3::new(0.80, 0.3, 0.4), 0.03);
        assert!(!stability.provisional_ready(0.03));
        stability.observe(Point3::new(0.98, 0.3, 0.4), 0.05);
        assert!(stability.provisional_ready(0.05));

        stability.observe(Point3::new(1.20, 0.3, 0.4), 0.09);
        assert!(stability.provisional_ready(0.09));
        stability.observe(Point3::new(0.70, 0.3, 0.4), 0.10);
        assert!(!stability.provisional_ready(0.10));
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
            trajectory.peak_rail_acceleration() <= crate::defaults::motion::RAIL_ACCEL_M_S2 + 1e-9
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
        assert!(
            candidates.len() <= 23,
            "확장 접수 창의 IK 시도 후보는 2.5cm 균등 표본 23개 이하"
        );
        let planned = PositionController::plan_best(&robot.arm, &start, &trajectory, &selector)
            .expect("기본 sim 샷의 위치 목표는 실행 가능해야 함");
        assert!(planned.arrives_on_time);
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

    #[test]
    fn sim_best_effort_moves_to_reachable_target_even_when_arrival_is_impossible() {
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
        )
        .into_iter()
        .map(|sample| TrajectorySample::new(sample.position, sample.velocity, 1e-6))
        .collect();
        let trajectory = BallTrajectory::new(vec![], predicted, Instant::now()).unwrap();
        let window = crate::robot::motion::InterceptWindow::default();
        let selector = HitTargetSelector::new(window.y_min, window.y_max).unwrap();

        let planned =
            PositionController::plan_best_or_reachable(&robot.arm, &start, &trajectory, &selector)
                .expect("시간은 놓쳐도 도달 가능한 후보로 움직여야 함");
        assert!(!planned.arrives_on_time);
        assert!(planned.trajectory.duration_secs > 0.0);
    }
}
