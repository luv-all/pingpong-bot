//! 다중 뷰 삼각측량 — OpenCV `triangulatePoints` + DLT 폴백.
//!
//! 순수 다중 뷰 기하라 카메라 쪽에 둔다. 추정기는 이제 픽셀을 직접 먹으므로
//! ([`crate::vision::Ekf`]) 삼각측량은 시드·캘리브 검증·물리 계측에서만 쓴다.

use nalgebra::{DMatrix, Matrix3x4};
use opencv::core::{CV_64F, Mat, MatTraitConst, Point2d, Vector};
use opencv::prelude::*;

use crate::Point3;
use crate::camera::{self, Calibration};

pub struct Triangulate;

impl Triangulate {
    /// 2뷰는 OpenCV, 3뷰 이상은 DLT (OpenCV stereo API가 2뷰 전용이라).
    pub fn views(views: &[(Matrix3x4<f64>, camera::Pixel)]) -> Option<Point3> {
        if views.len() < 2 {
            return None;
        }
        if views.len() == 2 {
            return two(&views[0], &views[1]).or_else(|| Self::dlt(views));
        }
        return Self::dlt(views);
    }

    /// 캘리브 + 픽셀 히트 → 월드 점. 카메라 수나 params가 모자라면 `None`.
    pub fn pixels(
        hits: &[(camera::Id, camera::Pixel)],
        calibration: &Calibration,
    ) -> Option<Point3> {
        if hits.len() < calibration.min_cameras_for_triangulation() {
            return None;
        }
        let views: Vec<_> = hits
            .iter()
            .map(|&(id, pixel)| {
                calibration
                    .params(id)
                    .map(|params| (params.projection_matrix(), pixel))
            })
            .collect::<Option<_>>()?;
        return Self::views(&views);
    }

    /// 동차 SVD.
    pub fn dlt(views: &[(Matrix3x4<f64>, camera::Pixel)]) -> Option<Point3> {
        if views.len() < 2 {
            return None;
        }
        let mut a = DMatrix::<f64>::zeros(2 * views.len(), 4);
        for (i, (p, pixel)) in views.iter().enumerate() {
            a.set_row(2 * i, &(p.row(2) * pixel.x - p.row(0)));
            a.set_row(2 * i + 1, &(p.row(2) * pixel.y - p.row(1)));
        }
        let svd = a.svd(true, true);
        let v_t = svd.v_t.as_ref()?;
        let h = v_t.row(v_t.nrows() - 1);
        return dehomogenise(h[0], h[1], h[2], h[3]);
    }

    /// 점을 카메라들로 투영했다가 되복원한다. 캘리브 자체의 기하 일관성 검사용.
    pub fn projections(
        calibration: &Calibration,
        camera_ids: &[camera::Id],
        point: Point3,
    ) -> Option<Point3> {
        let mut views = Vec::new();
        for id in camera_ids {
            let params: &camera::Params = calibration.params(*id)?;
            views.push((params.projection_matrix(), params.project_world(point)?));
        }
        return Self::views(&views);
    }
}

fn two(a: &(Matrix3x4<f64>, camera::Pixel), b: &(Matrix3x4<f64>, camera::Pixel)) -> Option<Point3> {
    let (proj_a, proj_b) = (to_mat(&a.0)?, to_mat(&b.0)?);
    let mut points_a = Vector::<Point2d>::new();
    points_a.push(Point2d::new(a.1.x, a.1.y));
    let mut points_b = Vector::<Point2d>::new();
    points_b.push(Point2d::new(b.1.x, b.1.y));

    let mut homogeneous = Mat::default();
    opencv::calib3d::triangulate_points(&proj_a, &proj_b, &points_a, &points_b, &mut homogeneous)
        .ok()?;
    let at = |row| homogeneous.at_2d::<f64>(row, 0).ok().copied();
    return dehomogenise(at(0)?, at(1)?, at(2)?, at(3)?);
}

fn dehomogenise(x: f64, y: f64, z: f64, w: f64) -> Option<Point3> {
    if !w.is_finite() || w.abs() < 1e-12 {
        return None;
    }
    let point = Point3::new(x / w, y / w, z / w);
    return point.coords.iter().all(|c| c.is_finite()).then_some(point);
}

fn to_mat(m: &Matrix3x4<f64>) -> Option<Mat> {
    let mut mat = Mat::zeros(3, 4, CV_64F).ok()?.to_mat().ok()?;
    for r in 0..3 {
        for c in 0..4 {
            *mat.at_2d_mut::<f64>(r, c).ok()? = m[(r as usize, c as usize)];
        }
    }
    return Some(mat);
}

#[cfg(test)]
#[path = "triangulate_tests.rs"]
mod tests;
