//! 다중 카메라 삼각측량 (1단계 스텁).
//!
//! - `sample_at`: 타임스탬프 동기화를 위한 선형 보간 (plan §5.4)
//! - `triangulate_synced`: 동기화 시각에 N대 관측을 맞춘 뒤 3D 복원 (DLT는 2단계)

use std::time::Instant;

use crate::error::{DomainError, ObservationError};
use crate::types::{BallObservation, Calibration, CameraId, PixelPoint, Point3, World};

/// 관측 시계열에서 `sync_time`에 해당하는 픽셀 위치를 선형 보간한다.
pub fn sample_at(observations: &[BallObservation], sync_time: Instant) -> Option<PixelPoint> {
    if observations.is_empty() {
        return None;
    }

    let first = observations.first()?;
    if sync_time <= first.timestamp {
        return Some(first.pixel);
    }

    let last = observations.last()?;
    if sync_time >= last.timestamp {
        return Some(last.pixel);
    }

    for window in observations.windows(2) {
        let earlier = &window[0];
        let later = &window[1];
        if earlier.timestamp <= sync_time && sync_time <= later.timestamp {
            let dt = (later.timestamp - earlier.timestamp).as_secs_f64();
            if dt <= f64::EPSILON {
                return Some(earlier.pixel);
            }
            let weight = (sync_time - earlier.timestamp).as_secs_f64() / dt;
            return Some(earlier.pixel.lerp(later.pixel, weight));
        }
    }

    return None;
}

/// 카메라별 관측 스트림을 `sync_time`으로 정렬한 뒤 3D 위치를 복원한다.
pub fn triangulate_synced(
    observations_by_camera: &[(CameraId, &[BallObservation])],
    sync_time: Instant,
    calibration: &Calibration,
) -> Result<Point3<World>, DomainError> {
    let required = calibration.min_cameras_for_triangulation();
    if observations_by_camera.len() < required {
        return Err(DomainError::InvalidObservation(
            ObservationError::TriangulationInsufficient {
                cameras_with_observation: observations_by_camera.len(),
                required,
            },
        ));
    }

    let mut samples = Vec::with_capacity(observations_by_camera.len());
    for (camera_id, series) in observations_by_camera {
        let pixel = sample_at(series, sync_time).ok_or_else(|| {
            DomainError::InvalidObservation(ObservationError::InterpolationFailed {
                camera_id: *camera_id,
            })
        })?;
        samples.push((*camera_id, pixel));
    }

    let mean_x = samples.iter().map(|(_, p)| p.x).sum::<f64>() / samples.len() as f64;
    let mean_y = samples.iter().map(|(_, p)| p.y).sum::<f64>() / samples.len() as f64;
    let spread = samples
        .first()
        .zip(samples.last())
        .map(|((_, a), (_, b))| (a.x - b.x).abs())
        .unwrap_or(0.0);

    return Ok(Point3::new(
        spread * 1e-3,
        mean_x * 1e-4,
        1.0 + mean_y * 1e-4,
    ));
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::*;
    use crate::types::CameraId;

    fn observation(
        camera_id: CameraId,
        base: Instant,
        elapsed_ms: u64,
        x: f64,
        y: f64,
    ) -> BallObservation {
        return BallObservation {
            pixel: PixelPoint::new(x, y),
            camera_id,
            timestamp: base + Duration::from_millis(elapsed_ms),
        };
    }

    #[test]
    fn sample_at_interpolates() {
        let base = Instant::now();
        let camera_id = CameraId::new(0);
        let series = vec![
            observation(camera_id, base, 0, 0.0, 0.0),
            observation(camera_id, base, 10, 10.0, 0.0),
        ];
        let mid = base + Duration::from_millis(5);
        let pixel = sample_at(&series, mid).expect("보간 결과");
        assert!((pixel.x - 5.0).abs() < 1e-9);
    }

    #[test]
    fn triangulate_requires_min_cameras() {
        let calibration = Calibration::default();
        let camera_id = CameraId::new(0);
        let series: [BallObservation; 0] = [];
        let err = triangulate_synced(&[(camera_id, &series[..])], Instant::now(), &calibration)
            .unwrap_err();
        assert!(matches!(
            err,
            DomainError::InvalidObservation(ObservationError::TriangulationInsufficient { .. })
        ));
    }
}
