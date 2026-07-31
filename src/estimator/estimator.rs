//! 공 상태 추정과 타격 평면 예측.

use std::time::Instant;

use crate::Point3;

use super::{BallTrajectory, HitPlane, Kinematics, Prediction};

pub trait Estimator: Send {
    fn update(&mut self, position: Point3, timestamp: Instant);
    /// 관측과 미래 예측을 같은 시간 기준으로 반환한다.
    fn trajectory(&self) -> Option<BallTrajectory>;

    /// 기존 제어와의 호환을 위한 평면 교차 어댑터.
    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        return Kinematics::prediction_from_trajectory(&self.trajectory()?, plane);
    }
}
