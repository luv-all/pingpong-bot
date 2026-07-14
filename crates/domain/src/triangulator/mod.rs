//! 다중 카메라 삼각측량.
//!
//! - `sample_at`: 타임스탬프 동기화를 위한 선형 보간
//! - `triangulate_synced`: 동기화 시각에 N대 관측을 맞춘 뒤 DLT로 3D 복원

use std::time::Instant;

use nalgebra::{DMatrix, Matrix3x4};

use crate::error::{DomainError, ObservationError};
use crate::types::{BallObservation, CameraId, PixelPoint, Point3};
use crate::camera::{Calibration, CameraParams};

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

/// 카메라별 관측 스트림을 `sync_time`으로 정렬한 뒤 DLT로 3D 위치를 복원한다.
pub fn triangulate_synced(
    observations_by_camera: &[(CameraId, &[BallObservation])],
    sync_time: Instant,
    calibration: &Calibration,
) -> Result<Point3, DomainError> {
    let required = calibration.min_cameras_for_triangulation();
    if observations_by_camera.len() < required {
        return Err(DomainError::InvalidObservation(
            ObservationError::TriangulationInsufficient {
                cameras_with_observation: observations_by_camera.len(),
                required,
            },
        ));
    }

    let mut views = Vec::with_capacity(observations_by_camera.len());
    for (camera_id, series) in observations_by_camera {
        let params = calibration.params(*camera_id).ok_or_else(|| {
            DomainError::InvalidObservation(ObservationError::MissingCalibration {
                camera_id: *camera_id,
            })
        })?;
        let pixel = sample_at(series, sync_time).ok_or_else(|| {
            DomainError::InvalidObservation(ObservationError::InterpolationFailed {
                camera_id: *camera_id,
            })
        })?;
        views.push((params.projection_matrix(), pixel));
    }

    return dlt_triangulate(&views).ok_or(DomainError::InvalidObservation(
        ObservationError::TriangulationFailed,
    ));
}

/// 알려진 픽셀/투영행렬로 DLT 삼각측량 (동차 SVD).
pub fn dlt_triangulate(views: &[(Matrix3x4<f64>, PixelPoint)]) -> Option<Point3> {
    if views.len() < 2 {
        return None;
    }

    let mut a = DMatrix::<f64>::zeros(2 * views.len(), 4);
    for (i, (p, pix)) in views.iter().enumerate() {
        let p1 = p.row(0);
        let p2 = p.row(1);
        let p3 = p.row(2);
        let row_u = p3 * pix.x - p1;
        let row_v = p3 * pix.y - p2;
        a.set_row(2 * i, &row_u);
        a.set_row(2 * i + 1, &row_v);
    }

    let svd = a.svd(true, true);
    let v_t = svd.v_t.as_ref()?;
    let h = v_t.row(v_t.nrows() - 1);
    let w = h[3];
    if !w.is_finite() || w.abs() < 1e-12 {
        return None;
    }
    let x = h[0] / w;
    let y = h[1] / w;
    let z = h[2] / w;
    if !(x.is_finite() && y.is_finite() && z.is_finite()) {
        return None;
    }
    return Some(Point3::new(x, y, z));
}

/// 테스트/디버그용: Calibration의 카메라들로 점을 투영한 뒤 DLT로 복원한다.
pub fn triangulate_projections(
    calibration: &Calibration,
    camera_ids: &[CameraId],
    point: Point3,
) -> Option<Point3> {
    let mut views = Vec::new();
    for id in camera_ids {
        let params: &CameraParams = calibration.params(*id)?;
        let pixel = params.project_world(point)?;
        views.push((params.projection_matrix(), pixel));
    }
    return dlt_triangulate(&views);
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::*;
    use crate::constants::table;
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

    #[test]
    fn dlt_recovers_known_point_noise_free() {
        let calibration = Calibration::sim(3);
        let truth = Point3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.4,
            table::SURFACE_Z + 0.2,
        );
        let ids = [
            CameraId::new(0),
            CameraId::new(1),
            CameraId::new(2),
        ];
        let recovered = triangulate_projections(&calibration, &ids, truth).expect("DLT");
        let err = (recovered.v - truth.v).norm();
        assert!(
            err < 1e-3,
            "noise-free DLT error {err} m (truth={:?} got={:?})",
            truth.v,
            recovered.v
        );
    }

    #[test]
    fn dlt_works_with_two_cameras() {
        let calibration = Calibration::sim(3);
        let truth = Point3::new(0.6, 1.0, 0.9);
        let recovered = triangulate_projections(
            &calibration,
            &[CameraId::new(0), CameraId::new(2)],
            truth,
        )
        .expect("2-view DLT");
        assert!((recovered.v - truth.v).norm() < 1e-3);
    }

    #[test]
    fn table_center_projects_near_image_center() {
        let cam = CameraParams::sim_layout(CameraId::new(1), 3);
        let pixel = cam
            .project_world(Point3::new(
                table::WIDTH_X * 0.5,
                table::LENGTH_Y * 0.5,
                table::SURFACE_Z,
            ))
            .expect("테이블 중앙");
        assert!((pixel.x - 320.0).abs() < 80.0);
        assert!((pixel.y - 240.0).abs() < 80.0);
    }

    #[test]
    fn triangulate_synced_recovers_from_series() {
        let calibration = Calibration::sim(3);
        let truth = Point3::new(0.7, 1.2, 1.0);
        let base = Instant::now();
        let mut series: Vec<(CameraId, Vec<BallObservation>)> = Vec::new();
        for id in [CameraId::new(0), CameraId::new(1), CameraId::new(2)] {
            let pixel = calibration
                .params(id)
                .unwrap()
                .project_world(truth)
                .expect("in view");
            series.push((
                id,
                vec![
                    observation(id, base, 0, pixel.x, pixel.y),
                    observation(id, base, 10, pixel.x, pixel.y),
                ],
            ));
        }
        let refs: Vec<(CameraId, &[BallObservation])> = series
            .iter()
            .map(|(id, s)| (*id, s.as_slice()))
            .collect();
        let mid = base + Duration::from_millis(5);
        let recovered = triangulate_synced(&refs, mid, &calibration).expect("synced DLT");
        assert!((recovered.v - truth.v).norm() < 1e-3);
    }
}
