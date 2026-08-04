//! 공 궤적에서 목표를 고르고 라켓을 그 위치까지 옮기는 공통 제어 경계.

use nalgebra::Vector3;
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

/// 실기와 시뮬레이션이 함께 사용하는 직접 제어 손목축 인덱스.
pub const DIRECT_WRIST_JOINT_INDEX: usize = 3;
/// 정밀 예측에서 준비 자세보다 더 움직이는 손목 각도.
pub const DIRECT_WRIST_STROKE_RAD: f64 = 15.0_f64.to_radians();
pub const MIN_DIRECT_COMMAND_SECS: f64 = 0.05;
pub const MAX_DIRECT_COMMAND_SECS: f64 = 0.30;

/// 공 예측 한 건에서 계산된 레일·손목 직접 명령.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectControlCommand {
    pub stage: PredictionStage,
    pub target: HitTarget,
    pub rail_x: f64,
    pub wrist_rad: f64,
    pub duration_secs: f64,
}

/// 명령 뒤 읽은 실제 위치와의 차이(`commanded - measured`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectControlMeasurement {
    pub rail_commanded_m: f64,
    pub rail_measured_m: f64,
    pub rail_error_m: f64,
    pub wrist_commanded_rad: f64,
    pub wrist_measured_rad: f64,
    pub wrist_error_rad: f64,
}

impl DirectControlCommand {
    pub fn compare_with_pose(&self, pose: &Pose) -> Option<DirectControlMeasurement> {
        let wrist_measured_rad = *pose.joints.values.get(DIRECT_WRIST_JOINT_INDEX)?;
        return Some(DirectControlMeasurement {
            rail_commanded_m: self.rail_x,
            rail_measured_m: pose.rail_x,
            rail_error_m: self.rail_x - pose.rail_x,
            wrist_commanded_rad: self.wrist_rad,
            wrist_measured_rad,
            wrist_error_rad: self.wrist_rad - wrist_measured_rad,
        });
    }
}

/// 전체 팔 궤적 대신 공의 x를 레일에, 예측 단계를 손목에 매핑하는 공통 제어기.
/// 실제 장치와 GUI 시뮬레이션 모두 이 계산 결과를 각 하드웨어 어댑터에 적용한다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectController {
    selector: HitTargetSelector,
    ready_wrist_rad: f64,
}

impl DirectController {
    pub fn new(y_min: f64, y_max: f64, ready_wrist_rad: f64) -> Result<Self, DirectControlError> {
        if !ready_wrist_rad.is_finite() {
            return Err(DirectControlError::InvalidReadyWrist);
        }
        let selector = HitTargetSelector::new(y_min, y_max).map_err(DirectControlError::Target)?;
        return Ok(Self {
            selector,
            ready_wrist_rad,
        });
    }

    /// `elapsed_secs`는 예측 기준 시각부터 명령 계산까지 흐른 시간이다.
    pub fn command(
        &self,
        arm: &Arm,
        trajectory: &BallTrajectory,
        stage: PredictionStage,
        elapsed_secs: f64,
    ) -> Result<DirectControlCommand, DirectControlError> {
        if !elapsed_secs.is_finite() || elapsed_secs < 0.0 {
            return Err(DirectControlError::InvalidElapsed);
        }
        let target = self
            .selector
            .select(trajectory)
            .map_err(DirectControlError::Target)?;
        let remaining_secs = target.time_secs - elapsed_secs;
        if !remaining_secs.is_finite() || remaining_secs <= 0.0 {
            return Err(DirectControlError::Expired {
                late_by_secs: -remaining_secs,
            });
        }
        let rail_x = arm
            .rail
            .map_or(target.position.x, |rail| rail.clamp_x(target.position.x));
        let wrist_rad = match stage {
            PredictionStage::Provisional => self.ready_wrist_rad,
            PredictionStage::Refined => self.ready_wrist_rad + DIRECT_WRIST_STROKE_RAD,
        };
        return Ok(DirectControlCommand {
            stage,
            target,
            rail_x,
            wrist_rad,
            duration_secs: remaining_secs.clamp(MIN_DIRECT_COMMAND_SECS, MAX_DIRECT_COMMAND_SECS),
        });
    }
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

#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum DirectControlError {
    #[error("준비 손목 각도가 유효하지 않음")]
    InvalidReadyWrist,
    #[error("예측 기준 경과 시간이 유효하지 않음")]
    InvalidElapsed,
    #[error("목표 시각이 {late_by_secs:.3}s 지남")]
    Expired { late_by_secs: f64 },
    #[error("목표 선택 실패: {0}")]
    Target(TargetSelectionError),
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
    fn direct_controller_maps_the_same_prediction_to_rail_and_wrist() {
        let robot = crate::defaults::robot().unwrap();
        let trajectory = BallTrajectory::new(
            vec![],
            vec![
                TrajectorySample::new(Point3::new(0.2, 0.5, 0.4), Vector3::zeros(), 0.1),
                TrajectorySample::new(Point3::new(0.4, 0.1, 0.2), Vector3::zeros(), 0.3),
            ],
            Instant::now(),
        )
        .unwrap();
        let controller = DirectController::new(0.2, 0.4, -0.7).unwrap();

        let provisional = controller
            .command(&robot.arm, &trajectory, PredictionStage::Provisional, 0.05)
            .unwrap();
        let refined = controller
            .command(&robot.arm, &trajectory, PredictionStage::Refined, 0.05)
            .unwrap();

        assert!((provisional.rail_x - 0.3).abs() < 1e-12);
        assert!((provisional.wrist_rad + 0.7).abs() < 1e-12);
        assert!((provisional.duration_secs - 0.15).abs() < 1e-12);
        assert!((refined.wrist_rad - (-0.7 + DIRECT_WRIST_STROKE_RAD)).abs() < 1e-12);
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
            wrist_rad: -0.40,
            duration_secs: 0.1,
        };
        let pose = Pose::new(
            0.27,
            super::super::Joints::from_slice(&[0.0, 0.0, 0.0, -0.45]),
        );
        let measured = command.compare_with_pose(&pose).unwrap();

        assert!((measured.rail_error_m - 0.03).abs() < 1e-12);
        assert!((measured.wrist_error_rad - 0.05).abs() < 1e-12);
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
