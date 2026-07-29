use opencv::Result as CvResult;
use opencv::core::{Mat, Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use crate::Point3;
use crate::camera;
use nalgebra::Vector3;

/// 창 좌표 → 원본 이미지 좌표. `scale` ≤ 0 이거나 1이면 그대로.
pub fn unscale_xy(x: i32, y: i32, scale: f64) -> (i32, i32) {
    if scale <= 0.0 || (scale - 1.0).abs() < 1e-9 {
        return (x, y);
    }
    return (
        (x as f64 / scale).round() as i32,
        (y as f64 / scale).round() as i32,
    );
}

/// 여러 BGR 패널을 가로로 붙인다.
/// 높이가 다르면 **최대 높이**에 맞추고 부족한 쪽은 검정 패딩 (리사이즈 없음 → 손실 없음).
pub fn hstack_bgr(panels: &[Mat]) -> CvResult<Mat> {
    if panels.is_empty() {
        return Ok(Mat::default());
    }
    if panels.len() == 1 {
        return panels[0].try_clone();
    }
    let max_h = panels.iter().map(|p| p.rows()).max().unwrap_or(1).max(1);
    let mut padded = Vec::with_capacity(panels.len());
    for p in panels {
        if p.rows() == max_h {
            padded.push(p.try_clone()?);
            continue;
        }
        let mut canvas = Mat::zeros(max_h, p.cols(), p.typ())?.to_mat()?;
        let roi = opencv::core::Rect::new(0, 0, p.cols(), p.rows());
        let mut dst = Mat::roi_mut(&mut canvas, roi)?;
        p.copy_to(&mut dst)?;
        padded.push(canvas);
    }
    let mut mosaic = Mat::default();
    opencv::core::hconcat(&Vector::<Mat>::from_iter(padded), &mut mosaic)?;
    return Ok(mosaic);
}

/// 이미지 높이 기준 오버레이 스케일 (720p ≈ 1.0). Hershey는 유니코드 미지원.
/// 모자이크처럼 세로가 커져도 글자가 폭주하지 않게 상한을 낮춘다.
pub(super) fn overlay_scale(img_h: i32) -> f64 {
    return (img_h as f64 / 720.0).clamp(0.5, 1.0);
}

/// 검출/궤적 마커 원.
pub fn draw_circle_px(
    img: &mut Mat,
    pixel: camera::Pixel,
    radius: i32,
    color: Scalar,
    thickness: i32,
) -> CvResult<()> {
    imgproc::circle(
        img,
        Point::new(pixel.x.round() as i32, pixel.y.round() as i32),
        radius,
        color,
        thickness,
        imgproc::LINE_8,
        0,
    )?;
    return Ok(());
}

/// 월드 점·속도를 카메라에 투영해 화살표를 그린다. `dt_draw` 초만큼 전진한 끝을 tip으로.
pub fn draw_world_velocity(
    img: &mut Mat,
    params: &camera::Params,
    origin: Point3,
    vel: Vector3<f64>,
    dt_draw: f64,
    color: Scalar,
) -> CvResult<()> {
    let Some(from) = params.project_world(origin) else {
        return Ok(());
    };
    let tip = Point3::from(origin.coords + vel * dt_draw);
    let Some(to) = params.project_world(tip) else {
        return draw_circle_px(img, from, 6, color, 2);
    };
    imgproc::arrowed_line(
        img,
        Point::new(from.x.round() as i32, from.y.round() as i32),
        Point::new(to.x.round() as i32, to.y.round() as i32),
        color,
        2,
        imgproc::LINE_8,
        0,
        0.25,
    )?;
    return Ok(());
}

/// 패널 한 장에 카메라 라벨.
pub fn draw_cam_label(img: &mut Mat, label: &str, color: Scalar) -> CvResult<()> {
    let s = overlay_scale(img.rows());
    let font_scale = 0.9 * s;
    let thickness = (2.0 * s).round().max(2.0) as i32;
    let margin = (18.0 * s).round() as i32;
    imgproc::put_text(
        img,
        label,
        Point::new(margin, img.rows().saturating_sub(margin).max(margin + 8)),
        imgproc::FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        thickness,
        imgproc::LINE_8,
        false,
    )?;
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unscale_xy_roundtrips_at_half() {
        let (x, y) = unscale_xy(500, 200, 0.5);
        assert_eq!((x, y), (1000, 400));
        assert_eq!(unscale_xy(10, 20, 1.0), (10, 20));
    }
}
