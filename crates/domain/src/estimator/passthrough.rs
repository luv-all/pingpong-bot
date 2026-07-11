//! 궤적 추정기 — [`PassThroughEstimator`]는 레거시 스텁.
//!
//! 본선은 [`super::ekf::BallEkf`].

use std::time::Instant;

use nalgebra::Vector3;

use super::ballistics::predict_hit_plane;
use crate::ports::Estimator;
use crate::types::{HitPlane, Point3, Prediction, World};

/// 테스트용 단순 추정기 — 관측 위치를 그대로 쓰고 고정 속도로 hit-plane을 예측한다.
pub struct PassThroughEstimator {
    last: Option<Point3<World>>,
    last_time: Option<Instant>,
    drag_coefficient: f64,
}

impl PassThroughEstimator {
    /// 저항 계수를 지정해 생성한다.
    pub fn new(drag_coefficient: f64) -> Self {
        return Self {
            last: None,
            last_time: None,
            drag_coefficient,
        };
    }

    /// 설정된 저항 계수.
    pub fn drag_coefficient(&self) -> f64 {
        return self.drag_coefficient;
    }
}

impl Estimator for PassThroughEstimator {
    fn update(&mut self, position: Point3<World>, timestamp: Instant) {
        self.last = Some(position);
        self.last_time = Some(timestamp);
    }

    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        let position = self.last.as_ref()?.v;
        // 대략 로봇 쪽으로 오는 속도 가정
        let velocity = Vector3::new(0.0, -5.0, 0.0);
        return predict_hit_plane(position, velocity, plane, self.drag_coefficient).or_else(|| {
            Some(Prediction {
                time_to_impact_secs: 0.3,
                impact_position: Point3::new(position.x, plane.y, position.z),
                incoming_velocity: velocity,
            })
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pass_through_produces_prediction() {
        let mut estimator = PassThroughEstimator::new(0.0);
        estimator.update(
            Point3::new(0.7, 1.5, 0.9),
            Instant::now(),
        );
        let prediction = estimator
            .predict_to(HitPlane { y: 0.30 })
            .expect("예측값");
        assert!((prediction.impact_position.v.y - 0.30).abs() < 1e-4);
    }
}
