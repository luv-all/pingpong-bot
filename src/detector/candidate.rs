//! 검출 후보 — generator가 만들고 scorer가 고른다.

use opencv::core::{Point, Vector};
use opencv::imgproc;

use crate::PixelPoint;

/// 한 프레임 안의 공 후보.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub pixel: PixelPoint,
    pub area: f64,
    /// `4π·area / peri²`. 원일수록 1에 가깝다.
    pub circularity: f64,
    /// motion overlap·디버그용. generator가 채운다.
    pub contour: Vector<Point>,
}

/// OpenCV 컨투어 → [`Candidate`]. area/peri 실패 시 `None`.
pub fn candidate_from_contour(contour: &Vector<Point>) -> Option<Candidate> {
    let area = imgproc::contour_area(contour, false).ok()?;
    if !area.is_finite() || area <= 0.0 {
        return None;
    }
    let peri = imgproc::arc_length(contour, true).ok()?;
    if peri < f64::EPSILON {
        return None;
    }
    let circularity = 4.0 * std::f64::consts::PI * area / (peri * peri);
    let moments = imgproc::moments(contour, false).ok()?;
    if moments.m00.abs() < f64::EPSILON {
        return None;
    }
    return Some(Candidate {
        pixel: PixelPoint::new(moments.m10 / moments.m00, moments.m01 / moments.m00),
        area,
        circularity,
        contour: contour.clone(),
    });
}

/// 컨투어 목록 → 후보 목록 (필터 없음).
pub fn candidates_from_contours(contours: &Vector<Vector<Point>>) -> Vec<Candidate> {
    let mut out = Vec::new();
    for contour in contours.iter() {
        if let Some(c) = candidate_from_contour(&contour) {
            out.push(c);
        }
    }
    return out;
}
