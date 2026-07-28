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
    /// 그 외 키 (Space=32, 's'=115, 화살표=waitKeyEx 풀코드 등).
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
    // waitKeyEx: 화살표가 macOS/X11에서 풀 키코드로 온다 (waitKey+&0xff는 Left≡'Q' 충돌).
    let key = highgui::wait_key_ex(wait_ms.max(1))?;
    let action = if key < 0 {
        PreviewAction::Continue
    } else if key == 27 || key == i32::from(b'q') || key == i32::from(b'Q') {
        PreviewAction::Quit
    } else {
        PreviewAction::Key(key)
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

/// 탁구대 XY×Z 월드 격자 오버레이 파라미터.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldGridParams {
    /// XY 격자 간격 [m]
    pub xy_step: f64,
    /// Z 층 간격 [m] (테이블 면 위)
    pub z_step: f64,
    /// Z 층 수 (≥1). `k = 0..z_layers`, `z = SURFACE_Z + k * z_step`
    pub z_layers: u32,
}

impl Default for WorldGridParams {
    fn default() -> Self {
        return Self {
            xy_step: 0.10,
            z_step: 0.05,
            z_layers: 6,
        };
    }
}

impl WorldGridParams {
    /// 키 조절용 하한 클램프.
    pub fn clamp(self) -> Self {
        return Self {
            xy_step: self.xy_step.max(0.02),
            z_step: self.z_step.max(0.02),
            z_layers: self.z_layers.max(1),
        };
    }
}

/// Z 정규화 t∈[0,1] → BGR jet-like (낮음=빨강 → 높음=파랑/보라).
fn jet_bgr(t: f64) -> Scalar {
    let t = t.clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.25 {
        let u = t / 0.25;
        (1.0, u, 0.0)
    } else if t < 0.5 {
        let u = (t - 0.25) / 0.25;
        (1.0 - u, 1.0, 0.0)
    } else if t < 0.75 {
        let u = (t - 0.5) / 0.25;
        (0.0, 1.0, u)
    } else {
        let u = (t - 0.75) / 0.25;
        (u * 0.5, 1.0 - u, 1.0)
    };
    return Scalar::new(b * 255.0, g * 255.0, r * 255.0, 0.0);
}

fn project_grid_pt(params: &CameraParams, x: f64, y: f64, z: f64) -> Option<Point> {
    let px = params.project_world(Point3::new(x, y, z))?;
    return Some(Point::new(px.x.round() as i32, px.y.round() as i32));
}

/// 탁구대 XY×Z 격자를 `project_world`로 투영해 점+선분으로 그린다.
pub fn draw_world_grid(
    img: &mut Mat,
    params: &CameraParams,
    grid: WorldGridParams,
) -> CvResult<()> {
    use crate::constants::table;

    let grid = grid.clamp();
    let xy = grid.xy_step;
    let dz = grid.z_step;
    let layers = grid.z_layers;
    let s = overlay_scale(img.rows());
    let radius = (4.0 * s).round().max(2.0) as i32;
    let line_th = (1.0 * s).round().max(1.0) as i32;

    let xs: Vec<f64> = {
        let mut v = Vec::new();
        let mut x = 0.0;
        while x <= table::WIDTH_X + 1e-9 {
            v.push(x);
            x += xy;
        }
        v
    };
    let ys: Vec<f64> = {
        let mut v = Vec::new();
        let mut y = 0.0;
        while y <= table::LENGTH_Y + 1e-9 {
            v.push(y);
            y += xy;
        }
        v
    };

    for (ki, k) in (0..layers).enumerate() {
        let z = table::SURFACE_Z + f64::from(k) * dz;
        let t = if layers <= 1 {
            0.0
        } else {
            f64::from(k) / f64::from(layers - 1)
        };
        let color = jet_bgr(t);

        for (i, &x) in xs.iter().enumerate() {
            for (j, &y) in ys.iter().enumerate() {
                let Some(p0) = project_grid_pt(params, x, y, z) else {
                    continue;
                };
                if i + 1 < xs.len() {
                    if let Some(p1) = project_grid_pt(params, xs[i + 1], y, z) {
                        imgproc::line(img, p0, p1, color, line_th, imgproc::LINE_AA, 0)?;
                    }
                }
                if j + 1 < ys.len() {
                    if let Some(p1) = project_grid_pt(params, x, ys[j + 1], z) {
                        imgproc::line(img, p0, p1, color, line_th, imgproc::LINE_AA, 0)?;
                    }
                }
            }
        }

        if ki + 1 < layers as usize {
            let z2 = table::SURFACE_Z + f64::from(k + 1) * dz;
            for &x in &xs {
                for &y in &ys {
                    let Some(p0) = project_grid_pt(params, x, y, z) else {
                        continue;
                    };
                    if let Some(p1) = project_grid_pt(params, x, y, z2) {
                        imgproc::line(img, p0, p1, color, line_th, imgproc::LINE_AA, 0)?;
                    }
                }
            }
        }
    }

    for &x in &xs {
        for &y in &ys {
            for k in 0..layers {
                let z = table::SURFACE_Z + f64::from(k) * dz;
                let t = if layers <= 1 {
                    0.0
                } else {
                    f64::from(k) / f64::from(layers - 1)
                };
                let color = jet_bgr(t);
                if let Some(px) = params.project_world(Point3::new(x, y, z)) {
                    draw_circle_px(img, px, radius, color, -1)?;
                }
            }
        }
    }

    let lines = [
        "World to Camera".to_string(),
        format!(
            "xy={:.2} z={:.2} layers={}",
            grid.xy_step, grid.z_step, grid.z_layers
        ),
    ];
    draw_debug_lines(img, &lines, Scalar::new(0.0, 0.0, 255.0, 0.0))?;
    return Ok(());
}

/// 격자 키: `+/-` XY, `[]` layers, `.,` Z step.
pub fn apply_grid_key(grid: &mut WorldGridParams, key: i32) {
    const XY_DELTA: f64 = 0.02;
    const Z_DELTA: f64 = 0.02;
    match key {
        k if k == i32::from(b'=') || k == i32::from(b'+') => {
            grid.xy_step += XY_DELTA;
        }
        k if k == i32::from(b'-') => {
            grid.xy_step -= XY_DELTA;
        }
        k if k == i32::from(b']') => {
            grid.z_layers = grid.z_layers.saturating_add(1);
        }
        k if k == i32::from(b'[') => {
            grid.z_layers = grid.z_layers.saturating_sub(1);
        }
        k if k == i32::from(b'.') => {
            grid.z_step += Z_DELTA;
        }
        k if k == i32::from(b',') => {
            grid.z_step -= Z_DELTA;
        }
        _ => {}
    }
    *grid = grid.clamp();
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

/// highgui 마우스: LMB/Enter 픽 큐 + Shift/nudge loupe + 방향키·hjkl 1px.
///
/// 좌표 규약 (툴은 매 프레임 [`Self::sync`] 후 읽기):
/// - [`Self::drain_clicks`] / [`Self::hover`] / aim → **원본 이미지** 픽셀
/// - 마우스가 움직이면 aim을 마우스에 즉시 재동기화
/// - 마우스 정지 중 방향키/`hjkl`은 aim만 ±1px (원본 기준)
///
/// loupe는 Shift **또는** 키보드 nudge 중에 표시.
#[derive(Debug, Default, Clone)]
pub struct PixelPickMouse {
    clicks: Vec<(i32, i32)>,
    /// loupe 중심 (이미지 좌표). [`Self::sync`]·[`Self::nudge`]가 갱신.
    pub hover: Option<(i32, i32)>,
    pub shift: bool,
    mouse_win: Option<(i32, i32)>,
    /// 마우스 좌표가 바뀌면 true → 다음 sync에서 aim = mouse.
    mouse_moved: bool,
    pending_lmb: bool,
    aim_img: Option<(i32, i32)>,
    /// 키보드로 aim을 옮긴 뒤. 마우스 이동 시 해제.
    nudged: bool,
}

impl PixelPickMouse {
    /// `set_mouse_callback`에서 호출. Shift는 `EVENT_FLAG_SHIFTKEY`(크로스플랫폼).
    pub fn on_event(&mut self, event: i32, x: i32, y: i32, flags: i32) {
        self.shift = (flags & highgui::EVENT_FLAG_SHIFTKEY) != 0;
        let moved = self
            .mouse_win
            .map(|(mx, my)| mx != x || my != y)
            .unwrap_or(true);
        self.mouse_win = Some((x, y));
        if moved {
            self.mouse_moved = true;
        }
        if event == highgui::EVENT_LBUTTONDOWN {
            self.pending_lmb = true;
        }
    }

    /// 창→이미지 동기화. 매 프레임 `drain`/`hover` 읽기 **전에** 호출.
    pub fn sync(&mut self, scale: f64, img_w: i32, img_h: i32) {
        if img_w <= 0 || img_h <= 0 {
            return;
        }
        if self.mouse_moved {
            if let Some((wx, wy)) = self.mouse_win {
                let (ix, iy) = unscale_xy(wx, wy, scale);
                self.aim_img = Some((ix.clamp(0, img_w - 1), iy.clamp(0, img_h - 1)));
            }
            self.mouse_moved = false;
            self.nudged = false;
        } else if self.aim_img.is_none() {
            if let Some((wx, wy)) = self.mouse_win {
                let (ix, iy) = unscale_xy(wx, wy, scale);
                self.aim_img = Some((ix.clamp(0, img_w - 1), iy.clamp(0, img_h - 1)));
            }
        }
        if self.pending_lmb {
            if let Some(a) = self.aim_img {
                self.clicks.push(a);
            }
            self.pending_lmb = false;
        }
        self.hover = if self.shift || self.nudged {
            self.aim_img
        } else {
            None
        };
    }

    /// 원본 이미지 기준 1px 단위 nudge. aim이 아직 없으면 no-op.
    pub fn nudge(&mut self, dx: i32, dy: i32, img_w: i32, img_h: i32) {
        if img_w <= 0 || img_h <= 0 {
            return;
        }
        let Some((x, y)) = self.aim_img else {
            return;
        };
        self.aim_img = Some((
            (x + dx).clamp(0, img_w - 1),
            (y + dy).clamp(0, img_h - 1),
        ));
        self.nudged = true;
        self.hover = self.aim_img;
    }

    /// Enter 등: 현재 aim을 클릭 큐에 넣는다.
    pub fn confirm(&mut self) {
        if let Some(a) = self.aim_img {
            self.clicks.push(a);
        }
    }

    /// 이미지 좌표 클릭 큐.
    pub fn drain_clicks(&mut self) -> Vec<(i32, i32)> {
        return std::mem::take(&mut self.clicks);
    }

    pub fn clear_clicks(&mut self) {
        self.clicks.clear();
        self.pending_lmb = false;
    }
}

/// [`PreviewAction::Key`] (waitKeyEx) → 이미지 (dx, dy).
///
/// 백엔드마다 코드가 다르다:
/// - **Win32**: VK가 상위 16비트 (`0x25xxxx` …). `(key >> 16) & 0xff`로 매칭
/// - **Cocoa**: `0xF700`–`0xF703` (하위 16비트)
/// - **X11/GTK**: `0xFF51`–`0xFF54`
/// - `hjkl`: 어느 백엔드에서든 동작하는 폴백 (`s`는 툴 단축키라 WASD 미사용)
pub fn arrow_delta(key: i32) -> Option<(i32, i32)> {
    // Win32 VK_* (waitKeyEx: virtual-key << 16). Shift 등 수정자 비트는 무시.
    const VK_LEFT: i32 = 0x25;
    const VK_UP: i32 = 0x26;
    const VK_RIGHT: i32 = 0x27;
    const VK_DOWN: i32 = 0x28;
    let win_vk = (key >> 16) & 0xff;
    if let Some(d) = match win_vk {
        VK_LEFT => Some((-1, 0)),
        VK_RIGHT => Some((1, 0)),
        VK_UP => Some((0, -1)),
        VK_DOWN => Some((0, 1)),
        _ => None,
    } {
        return Some(d);
    }

    // macOS Cocoa / X11 — 하위 16비트 (상위 수정자 무시)
    const MAC_UP: i32 = 0xF700;
    const MAC_DOWN: i32 = 0xF701;
    const MAC_LEFT: i32 = 0xF702;
    const MAC_RIGHT: i32 = 0xF703;
    const XK_LEFT: i32 = 0xFF51;
    const XK_UP: i32 = 0xFF52;
    const XK_RIGHT: i32 = 0xFF53;
    const XK_DOWN: i32 = 0xFF54;
    let code = key & 0xffff;
    if let Some(d) = match code {
        MAC_LEFT | XK_LEFT => Some((-1, 0)),
        MAC_RIGHT | XK_RIGHT => Some((1, 0)),
        MAC_UP | XK_UP => Some((0, -1)),
        MAC_DOWN | XK_DOWN => Some((0, 1)),
        _ => None,
    } {
        return Some(d);
    }

    match key & 0xff {
        k if k == i32::from(b'h') || k == i32::from(b'H') => Some((-1, 0)),
        k if k == i32::from(b'l') || k == i32::from(b'L') => Some((1, 0)),
        k if k == i32::from(b'k') || k == i32::from(b'K') => Some((0, -1)),
        k if k == i32::from(b'j') || k == i32::from(b'J') => Some((0, 1)),
        _ => None,
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

    #[test]
    fn pixel_pick_mouse_nudge_then_mouse_resync() {
        let mut m = PixelPickMouse::default();
        m.on_event(highgui::EVENT_MOUSEMOVE, 10, 20, 0);
        m.sync(1.0, 100, 100);
        m.nudge(1, -1, 100, 100);

        m.on_event(highgui::EVENT_LBUTTONDOWN, 10, 20, 0);
        m.sync(1.0, 100, 100);
        assert_eq!(m.drain_clicks(), vec![(11, 19)]);

        m.on_event(highgui::EVENT_MOUSEMOVE, 50, 60, 0);
        m.sync(1.0, 100, 100);
        m.confirm();
        assert_eq!(m.drain_clicks(), vec![(50, 60)]);
    }

    #[test]
    fn pixel_pick_mouse_shift_hover_is_aim_image_coords() {
        let mut m = PixelPickMouse::default();
        m.on_event(
            highgui::EVENT_MOUSEMOVE,
            5,
            10,
            highgui::EVENT_FLAG_SHIFTKEY,
        );
        m.sync(0.5, 200, 200);
        assert_eq!(m.hover, Some((10, 20)));
        m.nudge(1, 0, 200, 200);
        assert_eq!(m.hover, Some((11, 20)));
    }

    #[test]
    fn pixel_pick_mouse_nudge_keeps_loupe_without_shift() {
        let mut m = PixelPickMouse::default();
        m.on_event(highgui::EVENT_MOUSEMOVE, 10, 20, 0);
        m.sync(1.0, 100, 100);
        assert_eq!(m.hover, None);
        m.nudge(1, 0, 100, 100);
        assert_eq!(m.hover, Some((11, 20)));
    }

    #[test]
    fn arrow_delta_win32_mac_x11_and_hjkl() {
        // Win32 waitKeyEx: VK << 16 (Shift 눌러도 동일 — 수정자 OR 없음)
        assert_eq!(arrow_delta(0x25 << 16), Some((-1, 0))); // Left 2424832
        assert_eq!(arrow_delta(0x26 << 16), Some((0, -1))); // Up
        assert_eq!(arrow_delta(0x27 << 16), Some((1, 0))); // Right
        assert_eq!(arrow_delta(0x28 << 16), Some((0, 1))); // Down
        assert_eq!(arrow_delta(0xF702), Some((-1, 0)));
        assert_eq!(arrow_delta(0xFF53), Some((1, 0)));
        assert_eq!(arrow_delta(0xF702 | 0x10000), Some((-1, 0)));
        assert_eq!(arrow_delta(i32::from(b'h')), Some((-1, 0)));
        assert_eq!(arrow_delta(i32::from(b'J')), Some((0, 1)));
        assert_eq!(arrow_delta(i32::from(b'q')), None);
    }
}
