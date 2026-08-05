//! 선택된 예측 + quintic 궤적.

use crate::robot::motion::Prediction;

use super::trajectory::Trajectory;

#[derive(Debug, Clone, PartialEq)]
pub struct PlannedIntercept {
    pub prediction: Prediction,
    pub trajectory: Trajectory,
}
