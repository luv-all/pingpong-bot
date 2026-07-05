//! 궤적 추정기 (1단계: PassThrough 스텁).
//!
//! 확장 칼만 필터/RK4 본체는 2단계에서 `Estimator` trait의 다른 구현으로 교체한다.

use nalgebra::Vector3;

use crate::ports::Estimator;
use crate::types::{BallObservation, CameraId, HitPlane, Point3, Prediction, World};

/// sim/테스트용 추정기 — 관측 픽셀을 카메라별 오프셋으로 3D에 투영한다.
pub struct PassThroughEstimator {
    /// 마지막 3D 추정 위치
    last: Option<Point3<World>>,
    /// 공기 저항 계수 (2단계 EKF용)
    drag_coefficient: f64,
}

impl PassThroughEstimator {
    /// 저항 계수를 지정해 생성한다.
    pub fn new(drag_coefficient: f64) -> Self {
        return Self {
            last: None,
            drag_coefficient,
        };
    }

    /// 설정된 저항 계수.
    pub fn drag_coefficient(&self) -> f64 {
        return self.drag_coefficient;
    }

    /// 카메라 ID별 3D 오프셋.
    fn camera_offset(camera_id: CameraId) -> Vector3<f64> {
        let index = f64::from(camera_id.index());
        return Vector3::new((index - 1.0) * 0.5, 0.0, 1.0);
    }

    /// 관측 1건을 3D 점으로 변환한다.
    fn observation_to_point(observation: BallObservation) -> Point3<World> {
        let offset = Self::camera_offset(observation.camera_id);
        return Point3::from_vector(
            offset
                + Vector3::new(
                    observation.pixel.x * 1e-4,
                    observation.pixel.y * 1e-4,
                    0.0,
                ),
        );
    }
}

impl Estimator for PassThroughEstimator {
    fn update(&mut self, observation: BallObservation) {
        self.last = Some(Self::observation_to_point(observation));
    }

    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        let position = self.last.as_ref()?;
        return Some(Prediction {
            time_to_impact_secs: 0.3,
            impact_position: Point3::new(position.v.x, plane.y, position.v.z),
            incoming_velocity: Vector3::new(0.0, -1.0, 0.0),
        });
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;
    use crate::types::PixelPoint;

    #[test]
    fn pass_through_produces_prediction() {
        let mut estimator = PassThroughEstimator::new(0.01);
        estimator.update(BallObservation {
            pixel: PixelPoint::new(100.0, 200.0),
            camera_id: CameraId::new(0),
            timestamp: Instant::now(),
        });
        let prediction = estimator
            .predict_to(HitPlane { y: 1.0 })
            .expect("예측값");
        assert!((prediction.impact_position.v.y - 1.0).abs() < f64::EPSILON);
    }
}
