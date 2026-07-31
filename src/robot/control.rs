//! 공 궤적에서 목표를 고르고 라켓을 그 위치까지 옮기는 공통 제어 경계.

use nalgebra::Vector3;
use thiserror::Error;

use crate::Point3;
use crate::estimator::{BallTrajectory, TrajectorySample};

use super::motion::{Planner, Trajectory};
use super::{Arm, IkSearch, Pose};

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
        let mut candidates: Vec<HitTarget> = trajectory
            .predicted
            .iter()
            .filter(|sample| sample.position.y >= self.y_min && sample.position.y <= self.y_max)
            .copied()
            .map(HitTarget::from)
            .collect();
        for y in [self.y_min, (self.y_min + self.y_max) * 0.5, self.y_max] {
            if let Some(candidate) = interpolate_at_y(&trajectory.predicted, y)
                && candidates
                    .iter()
                    .all(|existing| (existing.time_secs - candidate.time_secs).abs() > 1e-9)
            {
                candidates.push(candidate);
            }
        }
        if candidates.is_empty() {
            return Err(TargetSelectionError::OutsideWindow);
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

/// fly_07·08·09에서 최종 도달점과 10 cm 이내로 계속 유지되기 시작한
/// 시점은 첫 정상 3D 관측 후 0.23~0.27 s였다. 중앙값 0.25 s와 실제
/// 수렴폭 10 cm를 둘 다 만족해야 정밀 예측으로 올린다.
pub const REFINED_MIN_OBSERVATION_SECS: f64 = 0.25;
pub const REFINED_TARGET_TOLERANCE_M: f64 = 0.10;
const REFINED_STABLE_SAMPLES: usize = 3;

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

        // 스윙 방향 계산은 하지 않는다. 상대 코트를 향하는 고정 안전 법선만 사용한다.
        let safe_normal = Vector3::new(0.0, 1.0, 0.0);
        // 실기에서는 바로 전 자세가 가장 좋은 IK 시드다. 빠른 지역 탐색을
        // 먼저 쓰고, 그 분기에 해가 없을 때만 전역 시드를 훑어 도달률을 보존한다.
        let goal = arm
            .inverse_pose_with_rail_best_normal(
                target.position,
                safe_normal,
                start,
                IkSearch::Local,
            )
            .or_else(|_| {
                arm.inverse_pose_with_rail_best_normal(
                    target.position,
                    safe_normal,
                    start,
                    IkSearch::Global,
                )
            })
            .map(|(pose, _normal_error)| pose)
            .map_err(|error| PositionControlError::Unreachable(error.to_string()))?;
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
    fn refined_stage_needs_time_and_three_targets_within_ten_centimeters() {
        let mut stability = PredictionStability::default();
        assert_eq!(
            stability.observe(Point3::new(0.0, 0.3, 0.4), 0.10),
            PredictionStage::Provisional
        );
        assert_eq!(
            stability.observe(Point3::new(0.04, 0.3, 0.4), 0.24),
            PredictionStage::Provisional
        );
        assert_eq!(
            stability.observe(Point3::new(0.05, 0.3, 0.4), 0.25),
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
        PositionController::plan_best(&robot.arm, &start, &trajectory, &selector)
            .expect("기본 sim 샷의 위치 목표는 실행 가능해야 함");
    }
}
