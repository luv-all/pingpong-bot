//! OpenCV highgui 프리뷰·디버그 오버레이 (detect/measure 툴 공용).

use opencv::core::{Mat, Point, Scalar, Vector};
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
fn overlay_scale(img_h: i32) -> f64 {
    return (img_h as f64 / 720.0).clamp(0.5, 1.0);
}

struct TextBlock {
    font_scale: f64,
    line_h: i32,
    pad: i32,
    outline: i32,
    fill: i32,
    max_w: i32,
    max_baseline: i32,
}

fn measure_text_block(
    lines: &[impl AsRef<str>],
    font_scale: f64,
    fill: i32,
) -> CvResult<(i32, i32, i32)> {
    let mut max_w = 0i32;
    let mut max_h = 0i32;
    let mut max_baseline = 0i32;
    for line in lines {
        let mut baseline = 0;
        let size = imgproc::get_text_size(
            line.as_ref(),
            imgproc::FONT_HERSHEY_SIMPLEX,
            font_scale,
            fill,
            &mut baseline,
        )?;
        max_w = max_w.max(size.width);
        max_h = max_h.max(size.height);
        max_baseline = max_baseline.max(baseline);
    }
    return Ok((max_w, max_h, max_baseline));
}

/// 가로·세로가 이미지 안에 들어오도록 font/line 스케일을 줄인다.
fn fit_text_block(
    img_w: i32,
    img_h: i32,
    lines: &[impl AsRef<str>],
    base_font: f64,
    base_line_h: f64,
    base_pad: f64,
    base_outline: f64,
    base_fill: f64,
) -> CvResult<TextBlock> {
    let n = lines.len().max(1) as i32;
    let mut font_scale = base_font;
    let mut line_h = base_line_h;
    let mut pad = base_pad;
    let mut outline = base_outline;
    let mut fill = base_fill;
    let mut max_w = 0i32;
    let mut max_baseline = 0i32;

    for _ in 0..10 {
        let fill_i = fill.round().max(1.0) as i32;
        let outline_i = outline.round().max(2.0) as i32;
        let (w, h, baseline) = measure_text_block(lines, font_scale, fill_i)?;
        let line_h_i = line_h.round().max(h as f64 + 4.0).max(10.0);
        let pad_i = pad.round().max(4.0);
        // 외곽선·디센더 여유까지 포함해 가용 영역에 맞춘다.
        let need_w = w as f64 + outline_i as f64 * 2.0 + 4.0 + pad_i * 2.0;
        let need_h = pad_i + line_h_i * n as f64 + baseline as f64 + outline_i as f64;
        let avail_w = img_w.max(1) as f64;
        let avail_h = img_h.max(1) as f64;
        let sx = if need_w > avail_w {
            avail_w / need_w
        } else {
            1.0
        };
        let sy = if need_h > avail_h {
            avail_h / need_h
        } else {
            1.0
        };
        let shrink = sx.min(sy).clamp(0.15, 1.0);
        max_w = w;
        max_baseline = baseline;
        if shrink >= 0.98 {
            return Ok(TextBlock {
                font_scale,
                line_h: line_h_i.round() as i32,
                pad: pad_i.round() as i32,
                outline: outline_i,
                fill: fill_i,
                max_w: max_w + outline_i * 2 + 4,
                max_baseline,
            });
        }
        font_scale *= shrink;
        line_h *= shrink;
        pad *= shrink.sqrt(); // 패드는 덜 줄여 가독성 유지
        outline = (outline * shrink).max(1.5);
        fill = (fill * shrink).max(1.0);
    }

    let fill_i = fill.round().max(1.0) as i32;
    let outline_i = outline.round().max(1.0) as i32;
    return Ok(TextBlock {
        font_scale,
        line_h: line_h.round().max(10.0) as i32,
        pad: pad.round().max(4.0) as i32,
        outline: outline_i,
        fill: fill_i,
        max_w: max_w + outline_i * 2 + 4,
        max_baseline,
    });
}

fn put_outlined_text(
    img: &mut Mat,
    text: &str,
    origin: Point,
    font_scale: f64,
    color: Scalar,
    outline: i32,
    fill: i32,
) -> CvResult<()> {
    imgproc::put_text(
        img,
        text,
        origin,
        imgproc::FONT_HERSHEY_SIMPLEX,
        font_scale,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        outline,
        imgproc::LINE_8,
        false,
    )?;
    imgproc::put_text(
        img,
        text,
        origin,
        imgproc::FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        fill,
        imgproc::LINE_8,
        false,
    )?;
    return Ok(());
}

/// 좌상단 디버그 텍스트 (검정 외곽 + 본문색). Hershey는 ASCII만 — 호출측도 ASCII.
pub fn draw_debug_lines(img: &mut Mat, lines: &[impl AsRef<str>], color: Scalar) -> CvResult<()> {
    if lines.is_empty() {
        return Ok(());
    }
    let s = overlay_scale(img.rows());
    let layout = fit_text_block(
        img.cols(),
        img.rows(),
        lines,
        0.85 * s,
        36.0 * s,
        14.0 * s,
        4.0 * s,
        2.0 * s,
    )?;
    for (i, line) in lines.iter().enumerate() {
        let y = layout.pad + layout.line_h * (i as i32 + 1);
        let y = y.min(img.rows() - layout.max_baseline - layout.outline).max(layout.pad + 8);
        put_outlined_text(
            img,
            line.as_ref(),
            Point::new(layout.pad, y),
            layout.font_scale,
            color,
            layout.outline,
            layout.fill,
        )?;
    }
    return Ok(());
}

/// 우하단 도움말 (아래부터 쌓음). Hershey ASCII만. 폭·높이에 맞춰 스케일다운.
pub fn draw_help_lines(img: &mut Mat, lines: &[impl AsRef<str>], color: Scalar) -> CvResult<()> {
    if lines.is_empty() {
        return Ok(());
    }
    let s = overlay_scale(img.rows());
    let layout = fit_text_block(
        img.cols(),
        img.rows(),
        lines,
        0.7 * s,
        30.0 * s,
        16.0 * s,
        3.0 * s,
        2.0 * s,
    )?;
    let n = lines.len() as i32;
    let x = (img.cols() - layout.pad - layout.max_w).max(layout.pad);
    // put_text y = baseline. 디센더·외곽선이 하단을 넘지 않게.
    let y_bottom = img.rows() - layout.pad - layout.max_baseline - layout.outline;
    let y_bottom = y_bottom.max(layout.pad + 8);

    for (i, line) in lines.iter().enumerate() {
        let y = y_bottom - layout.line_h * (n - 1 - i as i32);
        let y = y.max(layout.pad + 8);
        put_outlined_text(
            img,
            line.as_ref(),
            Point::new(x, y),
            layout.font_scale,
            color,
            layout.outline,
            layout.fill,
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
