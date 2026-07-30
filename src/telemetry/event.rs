//! 텔레메트리 이벤트.

use crate::estimator::Prediction;
use crate::motion;

#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryEvent {
    Prediction(Prediction),
    SwingCommand(motion::Trajectory),
}
