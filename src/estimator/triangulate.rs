//! 삼각측량 공개 진입점.

use std::time::Instant;

use crate::Point3;
use crate::camera;
use crate::camera::calib::Calibration;
use crate::detector;

/// 삼각측량 공개 진입점.
pub struct Triangulate;

impl Triangulate {
    pub fn sample_at(
        observations: &[detector::Observation],
        sync_time: Instant,
    ) -> Option<camera::Pixel> {
        return crate::estimator::tri::sample_at(observations, sync_time);
    }

    pub fn synced(
        observations_by_camera: &[(camera::Id, &[detector::Observation])],
        sync_time: Instant,
        calibration: &Calibration,
    ) -> Result<Point3, crate::error::DomainError> {
        return crate::estimator::tri::triangulate_synced(
            observations_by_camera,
            sync_time,
            calibration,
        );
    }

    pub fn views(views: &[(nalgebra::Matrix3x4<f64>, camera::Pixel)]) -> Option<Point3> {
        return crate::estimator::tri::triangulate_views(views);
    }

    /// 캘리브 + 픽셀 히트 → 월드 점. 카메라 수·params 부족하면 `None`.
    pub fn pixels(
        hits: &[(camera::Id, camera::Pixel)],
        calibration: &Calibration,
    ) -> Option<Point3> {
        if hits.len() < calibration.min_cameras_for_triangulation() {
            return None;
        }
        let views: Vec<_> = hits
            .iter()
            .map(|&(id, pix)| {
                calibration
                    .params(id)
                    .map(|params| (params.projection_matrix(), pix))
            })
            .collect::<Option<_>>()?;
        return Self::views(&views);
    }

    pub fn dlt(views: &[(nalgebra::Matrix3x4<f64>, camera::Pixel)]) -> Option<Point3> {
        return crate::estimator::tri::dlt_triangulate(views);
    }

    pub fn projections(
        calibration: &Calibration,
        camera_ids: &[camera::Id],
        point: Point3,
    ) -> Option<Point3> {
        return crate::estimator::tri::triangulate_projections(calibration, camera_ids, point);
    }
}
