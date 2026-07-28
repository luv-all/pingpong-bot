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

/// [`show_bgr`] 결과. `scale`은 디스플레이/원본 (항상 ≤ 1).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShowBgrResult {
    pub action: PreviewAction,
    pub scale: f64,
}

/// downscale 전용 fit 결과.
#[derive(Debug)]
pub struct FittedBgr {
    pub image: Mat,
    /// display = source * scale, 항상 ≤ 1.
    pub scale: f64,
}

/// 타이틀바·독 여유 (px).
const DISPLAY_FIT_MARGIN_PX: i32 = 96;

/// 모니터보다 클 때만 축소. 작으면 그대로(확대 없음).
pub fn fit_bgr_downscale(image: &Mat, max_w: i32, max_h: i32) -> CvResult<FittedBgr> {
    let w = image.cols();
    let h = image.rows();
    if w <= 0 || h <= 0 || max_w <= 0 || max_h <= 0 {
        return Ok(FittedBgr {
            image: image.try_clone()?,
            scale: 1.0,
        });
    }
    let scale = (max_w as f64 / w as f64)
        .min(max_h as f64 / h as f64)
        .min(1.0);
    if scale >= 1.0 - 1e-12 {
        return Ok(FittedBgr {
            image: image.try_clone()?,
            scale: 1.0,
        });
    }
    let nw = (w as f64 * scale).round().max(1.0) as i32;
    let nh = (h as f64 * scale).round().max(1.0) as i32;
    let mut out = Mat::default();
    imgproc::resize(
        image,
        &mut out,
        opencv::core::Size::new(nw, nh),
        0.0,
        0.0,
        imgproc::INTER_AREA,
    )?;
    return Ok(FittedBgr {
        image: out,
        scale: nw as f64 / w as f64,
    });
}

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

/// 주 디스플레이 작업 영역(여유 마진 제외). 실패 시 None → fit 생략.
pub fn display_fit_bounds() -> Option<(i32, i32)> {
    let (w, h) = primary_display_px()?;
    let max_w = (w - DISPLAY_FIT_MARGIN_PX).max(320);
    let max_h = (h - DISPLAY_FIT_MARGIN_PX).max(240);
    return Some((max_w, max_h));
}

fn primary_display_px() -> Option<(i32, i32)> {
    #[cfg(target_os = "macos")]
    {
        #[link(name = "CoreGraphics", kind = "framework")]
        unsafe extern "C" {
            fn CGMainDisplayID() -> u32;
            fn CGDisplayPixelsWide(display: u32) -> usize;
            fn CGDisplayPixelsHigh(display: u32) -> usize;
        }
        // SAFETY: CoreGraphics display query; no owned resources.
        unsafe {
            let id = CGMainDisplayID();
            let w = CGDisplayPixelsWide(id) as i32;
            let h = CGDisplayPixelsHigh(id) as i32;
            if w > 0 && h > 0 {
                return Some((w, h));
            }
        }
        return None;
    }
    #[cfg(target_os = "windows")]
    {
        #[link(name = "user32")]
        unsafe extern "system" {
            fn GetSystemMetrics(index: i32) -> i32;
        }
        // SAFETY: Win32 metrics; no owned resources.
        unsafe {
            let w = GetSystemMetrics(0); // SM_CXSCREEN
            let h = GetSystemMetrics(1); // SM_CYSCREEN
            if w > 0 && h > 0 {
                return Some((w, h));
            }
        }
        return None;
    }
    #[cfg(not(any(target_os = "macos", target_os = "windows")))]
    {
        return None;
    }
}

/// BGR 이미지를 창에 띄운다. 모니터보다 크면 downscale만 한다. `q` / ESC → Quit.
pub fn show_bgr(window: &str, image: &Mat, wait_ms: i32) -> CvResult<ShowBgrResult> {
    let fitted = match display_fit_bounds() {
        Some((max_w, max_h)) => fit_bgr_downscale(image, max_w, max_h)?,
        None => FittedBgr {
            image: image.try_clone()?,
            scale: 1.0,
        },
    };
    highgui::imshow(window, &fitted.image)?;
    let key = highgui::wait_key(wait_ms.max(1))?;
    let action = if key < 0 {
        PreviewAction::Continue
    } else {
        // macOS 등에서 상위 비트가 붙는 경우 대비
        let key = key & 0xff;
        if key == 27 || key == i32::from(b'q') || key == i32::from(b'Q') {
            PreviewAction::Quit
        } else {
            PreviewAction::Key(key)
        }
    };
    return Ok(ShowBgrResult {
        action,
        scale: fitted.scale,
    });
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

/// 픽셀 정밀 찍기용 loupe — [`crate::defaults::vision`].
pub use crate::defaults::vision::{PIXEL_LOUPE_SRC_HALF, PIXEL_LOUPE_ZOOM};

/// highgui 마우스: LMB 클릭 큐 + Shift-hold loupe 호버.
#[derive(Debug, Default, Clone)]
pub struct PixelPickMouse {
    pub clicks: Vec<(i32, i32)>,
    /// Shift 누른 채 마지막 커서 위치 (창 좌표).
    pub hover: Option<(i32, i32)>,
    pub shift: bool,
}

impl PixelPickMouse {
    /// `set_mouse_callback`에서 호출. Shift는 `EVENT_FLAG_SHIFTKEY`(크로스플랫폼).
    pub fn on_event(&mut self, event: i32, x: i32, y: i32, flags: i32) {
        self.shift = (flags & highgui::EVENT_FLAG_SHIFTKEY) != 0;
        if self.shift {
            self.hover = Some((x, y));
        } else {
            self.hover = None;
        }
        if event == highgui::EVENT_LBUTTONDOWN {
            self.clicks.push((x, y));
        }
    }

    pub fn drain_clicks(&mut self) -> Vec<(i32, i32)> {
        return std::mem::take(&mut self.clicks);
    }
}

/// `src`의 `(cx,cy)` 주변을 8× nearest로 확대해 `dst` 커서 위에 원형 loupe를 그린다.
///
/// `src`·`dst` 크기가 달라도 됨(모자이크 왼쪽 패널 등). 좌표는 둘 다 같은 원본 픽셀 기준.
/// 가장자리는 clamp 샘플. 중심 십자로 1px 정렬을 보이게 한다.
pub fn draw_pixel_loupe(dst: &mut Mat, src: &Mat, cx: i32, cy: i32) -> CvResult<()> {
    if src.empty() || dst.empty() || src.channels() != 3 || dst.channels() != 3 {
        return Ok(());
    }
    let sw = src.cols();
    let sh = src.rows();
    if sw <= 0 || sh <= 0 || cx < 0 || cy < 0 || cx >= sw || cy >= sh {
        return Ok(());
    }

    let half = PIXEL_LOUPE_SRC_HALF;
    let side = 2 * half + 1;
    let zoom = PIXEL_LOUPE_ZOOM;
    let out_side = side * zoom;
    let loupe_r = out_side / 2;

    let mut crop = Mat::zeros(side, side, src.typ())?.to_mat()?;
    for dy in -half..=half {
        for dx in -half..=half {
            let sx = (cx + dx).clamp(0, sw - 1);
            let sy = (cy + dy).clamp(0, sh - 1);
            let pix = *src.at_2d::<opencv::core::Vec3b>(sy, sx)?;
            *crop.at_2d_mut::<opencv::core::Vec3b>(dy + half, dx + half)? = pix;
        }
    }

    let mut zoomed = Mat::default();
    imgproc::resize(
        &crop,
        &mut zoomed,
        opencv::core::Size::new(out_side, out_side),
        0.0,
        0.0,
        imgproc::INTER_NEAREST,
    )?;

    let mut mask = Mat::zeros(out_side, out_side, opencv::core::CV_8UC1)?.to_mat()?;
    imgproc::circle(
        &mut mask,
        Point::new(loupe_r, loupe_r),
        loupe_r - 1,
        Scalar::all(255.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;

    let dw = dst.cols();
    let dh = dst.rows();
    let x0 = cx - loupe_r;
    let y0 = cy - loupe_r;
    for y in 0..out_side {
        let dy = y0 + y;
        if dy < 0 || dy >= dh {
            continue;
        }
        for x in 0..out_side {
            let dx = x0 + x;
            if dx < 0 || dx >= dw {
                continue;
            }
            if *mask.at_2d::<u8>(y, x)? == 0 {
                continue;
            }
            let pix = *zoomed.at_2d::<opencv::core::Vec3b>(y, x)?;
            *dst.at_2d_mut::<opencv::core::Vec3b>(dy, dx)? = pix;
        }
    }

    let center = Point::new(cx, cy);
    imgproc::circle(
        dst,
        center,
        loupe_r,
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        2,
        imgproc::LINE_AA,
        0,
    )?;
    // 중심 픽셀(확대 블록) 테두리
    let block = zoom / 2;
    imgproc::rectangle(
        dst,
        opencv::core::Rect::new(cx - block, cy - block, zoom, zoom),
        Scalar::new(0.0, 0.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    // 십자
    imgproc::line(
        dst,
        Point::new(cx - loupe_r + 4, cy),
        Point::new(cx - block - 2, cy),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        dst,
        Point::new(cx + block + 2, cy),
        Point::new(cx + loupe_r - 4, cy),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        dst,
        Point::new(cx, cy - loupe_r + 4),
        Point::new(cx, cy - block - 2),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        dst,
        Point::new(cx, cy + block + 2),
        Point::new(cx, cy + loupe_r - 4),
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        0,
    )?;

    let label = format!("{cx},{cy}");
    let tx = (cx - loupe_r).clamp(2, (dw - 80).max(2));
    let ty = (cy - loupe_r - 6).clamp(14, (dh - 2).max(14));
    imgproc::put_text(
        dst,
        &label,
        Point::new(tx, ty),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.45,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        2,
        imgproc::LINE_AA,
        false,
    )?;
    imgproc::put_text(
        dst,
        &label,
        Point::new(tx, ty),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.45,
        Scalar::new(0.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    )?;
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bgr(w: i32, h: i32) -> Mat {
        return Mat::zeros(h, w, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
    }

    #[test]
    fn fit_downscale_keeps_small_image() {
        let img = bgr(100, 50);
        let fitted = fit_bgr_downscale(&img, 200, 200).unwrap();
        assert_eq!(fitted.image.cols(), 100);
        assert_eq!(fitted.image.rows(), 50);
        assert!((fitted.scale - 1.0).abs() < 1e-9);
    }

    #[test]
    fn fit_downscale_shrinks_preserving_aspect() {
        let img = bgr(2000, 1000);
        let fitted = fit_bgr_downscale(&img, 1000, 800).unwrap();
        assert_eq!(fitted.image.cols(), 1000);
        assert_eq!(fitted.image.rows(), 500);
        assert!((fitted.scale - 0.5).abs() < 1e-6);
    }

    #[test]
    fn unscale_xy_roundtrips_at_half() {
        let (x, y) = unscale_xy(500, 200, 0.5);
        assert_eq!((x, y), (1000, 400));
        assert_eq!(unscale_xy(10, 20, 1.0), (10, 20));
    }
}
