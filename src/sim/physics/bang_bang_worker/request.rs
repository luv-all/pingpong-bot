use std::sync::Arc;

use crate::robot;
use crate::{Arm, Prediction};

pub(super) struct Request {
    pub(super) id: u64,
    pub(super) arm: Arc<Arm>,
    pub(super) predictions: Vec<Prediction>,
    pub(super) start: robot::Pose,
}
