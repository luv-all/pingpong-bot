use opencv::Result as CvResult;
use opencv::core::{Mat, Point, Scalar};
use opencv::imgproc;
use opencv::prelude::*;

use super::ops::{hershey, overlay_scale};

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

/// 좌상단 디버그 텍스트 (검정 외곽 + 본문색). Hershey는 ASCII만 — 호출측도 ASCII로 쓴다.
pub fn draw_debug_lines(img: &mut Mat, lines: &[impl AsRef<str>], color: Scalar) -> CvResult<()> {
    if lines.is_empty() {
        return Ok(());
    }
    // 폭 계산 전에 바꿔야 `…`→`...`처럼 길어지는 글자가 칸을 넘지 않는다.
    let lines: Vec<_> = lines.iter().map(|l| hershey(l.as_ref())).collect();
    let lines = &lines[..];
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
        let y = y
            .min(img.rows() - layout.max_baseline - layout.outline)
            .max(layout.pad + 8);
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
    let lines: Vec<_> = lines.iter().map(|l| hershey(l.as_ref())).collect();
    let lines = &lines[..];
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
