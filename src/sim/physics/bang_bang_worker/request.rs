use std::sync::Arc;

use crate::robot;
use crate::robot::Arm;
use crate::robot::motion::Prediction;

pub(super) struct Request {
    pub(super) id: u64,
    pub(super) arm: Arc<Arm>,
    pub(super) predictions: Vec<Prediction>,
    pub(super) start: robot::Pose,
}
