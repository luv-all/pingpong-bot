//! 텔레메트리 이벤트.

use crate::robot::motion;
use crate::robot::motion::Prediction;

#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryEvent {
    Prediction(Prediction),
    SwingCommand(motion::Trajectory),
}
