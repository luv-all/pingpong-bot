//! OpenCV highgui 프리뷰·디버그 오버레이 (detect/measure 툴 공용).

use opencv::core::{Mat, Point, Scalar, Size, Vector};
use opencv::imgproc;
use opencv::prelude::*;
use opencv::{Result as CvResult, highgui};

use crate::{CameraParams, PixelPoint, Point3};
use nalgebra::Vector3;

/// 프리뷰 키 입력.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreviewAction {
    /// 키 없음 (timeout)
    Continue,
    /// `q` / ESC
    Quit,
    /// 그 외 키 (Space=32, 's'=115 등). OpenCV waitKey 코드.
    Key(i32),
}

/// BGR 이미지를 창에 띄운다. `q` / ESC → Quit.
pub fn show_bgr(window: &str, image: &Mat, wait_ms: i32) -> CvResult<PreviewAction> {
    highgui::imshow(window, image)?;
    let key = highgui::wait_key(wait_ms.max(1))?;
    if key < 0 {
        return Ok(PreviewAction::Continue);
    }
    // macOS 등에서 상위 비트가 붙는 경우 대비
    let key = key & 0xff;
    if key == 27 || key == i32::from(b'q') || key == i32::from(b'Q') {
        return Ok(PreviewAction::Quit);
    }
    return Ok(PreviewAction::Key(key));
}

/// 창을 닫는다 (프로세스 종료 전 호출 권장).
pub fn destroy_window(window: &str) {
    let _ = highgui::destroy_window(window);
}

/// 여러 BGR 패널을 가로로 붙인다. 높이가 다르면 최소 높이에 맞춘다.
pub fn hstack_bgr(panels: &[Mat]) -> CvResult<Mat> {
    if panels.is_empty() {
        return Ok(Mat::default());
    }
    if panels.len() == 1 {
        return panels[0].try_clone();
    }
    let min_h = panels.iter().map(|p| p.rows()).min().unwrap_or(1).max(1);
    let mut resized = Vec::with_capacity(panels.len());
    for p in panels {
        if p.rows() == min_h {
            resized.push(p.try_clone()?);
            continue;
        }
        let scale = f64::from(min_h) / f64::from(p.rows().max(1));
        let w = ((f64::from(p.cols()) * scale).round() as i32).max(1);
        let mut out = Mat::default();
        imgproc::resize(
            p,
            &mut out,
            Size::new(w, min_h),
            0.0,
            0.0,
            imgproc::INTER_AREA,
        )?;
        resized.push(out);
    }
    let mut mosaic = Mat::default();
    opencv::core::hconcat(&Vector::<Mat>::from_iter(resized), &mut mosaic)?;
    return Ok(mosaic);
}

/// 좌상단 디버그 텍스트 (검정 외곽 + 본문색).
pub fn draw_debug_lines(img: &mut Mat, lines: &[impl AsRef<str>], color: Scalar) -> CvResult<()> {
    let line_h = 18;
    let pad = 8;
    for (i, line) in lines.iter().enumerate() {
        let y = pad + line_h * (i as i32 + 1);
        let origin = Point::new(pad, y);
        imgproc::put_text(
            img,
            line.as_ref(),
            origin,
            imgproc::FONT_HERSHEY_SIMPLEX,
            0.48,
            Scalar::new(0.0, 0.0, 0.0, 0.0),
            3,
            imgproc::LINE_8,
            false,
        )?;
        imgproc::put_text(
            img,
            line.as_ref(),
            origin,
            imgproc::FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            imgproc::LINE_8,
            false,
        )?;
    }
    return Ok(());
}

/// 검출/궤적 마커 원.
pub fn draw_circle_px(
    img: &mut Mat,
    pixel: PixelPoint,
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

/// 픽셀에서 속도 방향을 화살표로 (이미지 평면 추정, 길이 = |v|*scale_px).
pub fn draw_arrow_px(
    img: &mut Mat,
    from: PixelPoint,
    dir_uv: (f64, f64),
    length_px: f64,
    color: Scalar,
) -> CvResult<()> {
    let norm = (dir_uv.0 * dir_uv.0 + dir_uv.1 * dir_uv.1).sqrt();
    if norm < 1e-9 || length_px < 1.0 {
        return Ok(());
    }
    let ux = dir_uv.0 / norm;
    let uy = dir_uv.1 / norm;
    let to = Point::new(
        (from.x + ux * length_px).round() as i32,
        (from.y + uy * length_px).round() as i32,
    );
    let from_pt = Point::new(from.x.round() as i32, from.y.round() as i32);
    imgproc::arrowed_line(img, from_pt, to, color, 2, imgproc::LINE_8, 0, 0.25)?;
    return Ok(());
}

/// 월드 점·속도를 카메라에 투영해 화살표를 그린다. `dt_draw` 초만큼 전진한 끝을 tip으로.
pub fn draw_world_velocity(
    img: &mut Mat,
    params: &CameraParams,
    origin: Point3,
    vel: Vector3<f64>,
    dt_draw: f64,
    color: Scalar,
) -> CvResult<()> {
    let Some(from) = params.project_world(origin) else {
        return Ok(());
    };
    let tip = Point3::from_vector(origin.v + vel * dt_draw);
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
    imgproc::put_text(
        img,
        label,
        Point::new(8, img.rows().saturating_sub(12).max(20)),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        imgproc::LINE_8,
        false,
    )?;
    return Ok(());
}
