//! 선택된 예측 + bang-bang 궤적.

use crate::error::{DomainError, SwingPlanError};
use crate::robot::motion::Prediction;
use crate::robot::{self, Arm};

use super::super::physics::in_swing_commit_window;
use super::guidance::plan_bang_bang_for;
use super::trajectory::Trajectory;

/// `predictions` 중 IK가 풀리는 첫 후보로 bang-bang 궤적을 계획한다.
/// 선택 순서는 `plan_best_swing`과 같은 "현재 라켓 위치에 가까운 순".
/// `plan_bang_bang_swing`이 실제로 고른 예측 + 궤적 - `PlannedIntercept`
/// (quintic)와 대응. GUI가 "어떤 hit-plane을 겨냥했는지" 디버그 표시에 쓴다.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedIntercept {
    pub prediction: Prediction,
    pub trajectory: Trajectory,
}

pub fn plan_bang_bang_swing(
    arm: &Arm,
    predictions: &[Prediction],
    start: &robot::Pose,
) -> Result<PlannedIntercept, DomainError> {
    let current_position = if arm.rail.is_some() {
        arm.forward_kinematics_with_rail(start.rail_x, &start.joints)
    } else {
        arm.forward_kinematics(&start.joints)
    }
    .map(|pose| pose.position.coords)
    .unwrap_or_default();
    let mut ranked: Vec<Prediction> = predictions
        .iter()
        .copied()
        .filter(|prediction| in_swing_commit_window(prediction.time_to_impact_secs))
        .collect();
    ranked.sort_by(|left, right| {
        let left_cost = (left.impact_position.coords - current_position).norm();
        let right_cost = (right.impact_position.coords - current_position).norm();
        left_cost
            .partial_cmp(&right_cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut last_error = None;
    for prediction in ranked {
        match plan_bang_bang_for(arm, &prediction, start) {
            Ok(trajectory) => {
                return Ok(PlannedIntercept {
                    prediction,
                    trajectory,
                });
            }
            Err(error) => last_error = Some(error),
        }
    }
    return Err(last_error.unwrap_or(DomainError::InfeasibleSwing(
        SwingPlanError::InsufficientTime {
            time_to_impact_secs: 0.0,
            min_swing_secs: 0.0,
        },
    )));
}
