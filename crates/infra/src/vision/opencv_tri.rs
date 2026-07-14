//! OpenCV `triangulatePoints` 경로 (feature = "opencv").

use nalgebra::Matrix3x4;
use opencv::core::{Mat, MatTraitConst, Point2d, Vector, CV_64F};
use opencv::prelude::*;
use pingpong_domain::{PixelPoint, Point3};

use super::triangulate::dlt_triangulate;

/// 2뷰는 OpenCV `triangulate_points`, 3뷰 이상은 nalgebra DLT(동일 수식)로 폴백.
pub fn triangulate_views(views: &[(Matrix3x4<f64>, PixelPoint)]) -> Option<Point3> {
    if views.len() < 2 {
        return None;
    }
    if views.len() == 2 {
        return triangulate_two(&views[0], &views[1]).or_else(|| dlt_triangulate(views));
    }
    // N>=3: 검증된 DLT (OpenCV stereo API는 2뷰 전용)
    return dlt_triangulate(views);
}

fn triangulate_two(
    a: &(Matrix3x4<f64>, PixelPoint),
    b: &(Matrix3x4<f64>, PixelPoint),
) -> Option<Point3> {
    let proj1 = matrix3x4_to_mat(&a.0)?;
    let proj2 = matrix3x4_to_mat(&b.0)?;
    let mut points1 = Vector::<Point2d>::new();
    points1.push(Point2d::new(a.1.x, a.1.y));
    let mut points2 = Vector::<Point2d>::new();
    points2.push(Point2d::new(b.1.x, b.1.y));

    let mut points4d = Mat::default();
    opencv::calib3d::triangulate_points(&proj1, &proj2, &points1, &points2, &mut points4d).ok()?;

    // 4x1 동차 -> 유클리드
    let x = *points4d.at_2d::<f64>(0, 0).ok()?;
    let y = *points4d.at_2d::<f64>(1, 0).ok()?;
    let z = *points4d.at_2d::<f64>(2, 0).ok()?;
    let w = *points4d.at_2d::<f64>(3, 0).ok()?;
    if !w.is_finite() || w.abs() < 1e-12 {
        return None;
    }
    let px = x / w;
    let py = y / w;
    let pz = z / w;
    if !(px.is_finite() && py.is_finite() && pz.is_finite()) {
        return None;
    }
    return Some(Point3::new(px, py, pz));
}

fn matrix3x4_to_mat(m: &Matrix3x4<f64>) -> Option<Mat> {
    let mut mat = Mat::zeros(3, 4, CV_64F).ok()?.to_mat().ok()?;
    for r in 0..3 {
        for c in 0..4 {
            *mat.at_2d_mut::<f64>(r, c).ok()? = m[(r as usize, c as usize)];
        }
    }
    return Some(mat);
}
