//! 공 상태 추정과 타격 평면 예측.

use std::time::Instant;

use crate::Point3;

use super::{HitPlane, Prediction};

pub trait Estimator: Send {
    fn update(&mut self, position: Point3, timestamp: Instant);
    fn predict_to(&self, plane: HitPlane) -> Option<Prediction>;
}
