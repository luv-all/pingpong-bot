//! 탁구공 색 범위 튜닝 — 픽커 → 퍼센타일 구간(+margin) → `data/colormask.json` upsert.
//!
//! 레이아웃: (original | mask) / swatch / scatter+iso. `p`·종료 시 `data/colormask.json` upsert.

mod cli;

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, bail};
use clap::Parser;
use opencv::core::{Rect, Scalar, Vec3b, Vector};
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::defaults::colormask_path;
use pingpong_bot::detector::{load_colormask_set_or_empty, save_colormask_set};
use pingpong_bot::{
    ColorSpace, ColormaskParams, FrameSource, Id, ImageDirSource, Pixel, PixelPickMouse, Preview,
    PreviewAction,
};

use cli::Args;

const SWATCH_H: i32 = 36;
const VIZ_H: i32 = 200;
const SAMPLE_RADIUS: i32 = 2;
const PLOT_PAD: i32 = 18;

#[derive(Clone, Copy, Debug)]
struct Sample {
    x: i32,
    y: i32,
    bgr: [u8; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ChannelRange {
    c0_min: u8,
    c0_max: u8,
    c1_min: u8,
    c1_max: u8,
    c2_min: u8,
    c2_max: u8,
}

/// 정렬된 채널 값에서 선형 보간 퍼센타일 (p ∈ [0, 100]).
fn channel_percentile(sorted: &[u8], p: f64) -> u8 {
    debug_assert!(!sorted.is_empty());
    if sorted.len() == 1 {
        return sorted[0];
    }
    let p = p.clamp(0.0, 100.0);
    let rank = p / 100.0 * (sorted.len() - 1) as f64;
    let lo = rank.floor() as usize;
    let hi = rank.ceil() as usize;
    if lo == hi {
        return sorted[lo];
    }
    let t = rank - lo as f64;
    return (f64::from(sorted[lo]) * (1.0 - t) + f64::from(sorted[hi]) * t).round() as u8;
}

impl ChannelRange {
    /// `trim_pct`: 양꼬리 절단 % (0 → min/max, 10 → p10..p90). 0..=49로 clamp.
    fn from_channels(chs: &[[u8; 3]], margin: u8, trim_pct: f64) -> Option<Self> {
        if chs.is_empty() {
            return None;
        }
        let trim = trim_pct.clamp(0.0, 49.0);
        let p_lo = trim;
        let p_hi = 100.0 - trim;
        let mut lo = [0u8; 3];
        let mut hi = [0u8; 3];
        for i in 0..3 {
            let mut vals: Vec<u8> = chs.iter().map(|c| c[i]).collect();
            vals.sort_unstable();
            lo[i] = channel_percentile(&vals, p_lo);
            hi[i] = channel_percentile(&vals, p_hi);
            if lo[i] > hi[i] {
                std::mem::swap(&mut lo[i], &mut hi[i]);
            }
        }
        return Some(Self {
            c0_min: lo[0].saturating_sub(margin),
            c0_max: hi[0].saturating_add(margin),
            c1_min: lo[1].saturating_sub(margin),
            c1_max: hi[1].saturating_add(margin),
            c2_min: lo[2].saturating_sub(margin),
            c2_max: hi[2].saturating_add(margin),
        });
    }

    fn to_params(self, space: ColorSpace) -> ColormaskParams {
        return ColormaskParams {
            space,
            c0_min: self.c0_min,
            c0_max: self.c0_max,
            c1_min: self.c1_min,
            c1_max: self.c1_max,
            c2_min: self.c2_min,
            c2_max: self.c2_max,
        };
    }
}

fn open_source(args: &Args) -> Result<Box<dyn FrameSource>> {
    let cam_id = args.cam.camera_id().map_err(anyhow::Error::msg)?;
    if let Some(images) = &args.images {
        if args.offline.has_offline() {
            bail!("--images 와 --clip 동시 사용 불가");
        }
        return Ok(Box::new(
            ImageDirSource::open(cam_id, images)
                .map_err(anyhow::Error::msg)
                .context("images")?,
        ));
    }
    return Ok(args
        .cam
        .open_mono_input(&args.offline)
        .map_err(anyhow::Error::msg)?);
}

fn read_bgr_avg(img: &Mat, x: i32, y: i32, radius: i32) -> Option<[u8; 3]> {
    let w = img.cols();
    let h = img.rows();
    if x < 0 || y < 0 || x >= w || y >= h {
        return None;
    }
    let mut sum = [0u32; 3];
    let mut n = 0u32;
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let px = x + dx;
            let py = y + dy;
            if px < 0 || py < 0 || px >= w || py >= h {
                continue;
            }
            let v: Vec3b = *img.at_2d(py, px).ok()?;
            sum[0] += u32::from(v[0]);
            sum[1] += u32::from(v[1]);
            sum[2] += u32::from(v[2]);
            n += 1;
        }
    }
    if n == 0 {
        return None;
    }
    return Some([(sum[0] / n) as u8, (sum[1] / n) as u8, (sum[2] / n) as u8]);
}

fn bgr_to_space(bgr: [u8; 3], space: ColorSpace) -> Result<[u8; 3]> {
    let pixel = Mat::new_rows_cols_with_default(
        1,
        1,
        opencv::core::CV_8UC3,
        Scalar::new(f64::from(bgr[0]), f64::from(bgr[1]), f64::from(bgr[2]), 0.0),
    )?;
    let mut out = Mat::default();
    let code = match space {
        ColorSpace::Ycrcb => imgproc::COLOR_BGR2YCrCb,
        ColorSpace::Hsv => imgproc::COLOR_BGR2HSV,
    };
    imgproc::cvt_color(
        &pixel,
        &mut out,
        code,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    let v: Vec3b = *out.at_2d(0, 0)?;
    return Ok([v[0], v[1], v[2]]);
}

fn ranges_from_samples(
    samples: &[Sample],
    margin: u8,
    trim_pct: f64,
) -> Result<(Option<ChannelRange>, Option<ChannelRange>)> {
    if samples.is_empty() {
        return Ok((None, None));
    }
    let mut ycrcb = Vec::with_capacity(samples.len());
    let mut hsv = Vec::with_capacity(samples.len());
    for s in samples {
        ycrcb.push(bgr_to_space(s.bgr, ColorSpace::Ycrcb)?);
        hsv.push(bgr_to_space(s.bgr, ColorSpace::Hsv)?);
    }
    return Ok((
        ChannelRange::from_channels(&ycrcb, margin, trim_pct),
        ChannelRange::from_channels(&hsv, margin, trim_pct),
    ));
}

fn space_label(space: ColorSpace) -> &'static str {
    return match space {
        ColorSpace::Ycrcb => "Y/Cr/Cb",
        ColorSpace::Hsv => "H/S/V",
    };
}

fn upsert_colormask(
    cam_id: Id,
    space: ColorSpace,
    range: ChannelRange,
    samples: &[Sample],
) -> Result<()> {
    let path = colormask_path();
    let mut set = load_colormask_set_or_empty(&path)?;
    let params = range.to_params(space);
    params.validate()?;
    let stored: Vec<[u8; 3]> = samples.iter().map(|s| s.bgr).collect();
    set.upsert(cam_id, params, stored);
    save_colormask_set(&path, &set)?;
    println!(
        "wrote colormask → {} (cam={}, space={}, samples={}, cams={})",
        path.display(),
        cam_id.0,
        space,
        samples.len(),
        set.cameras.len()
    );
    return Ok(());
}

fn load_samples_for_cam(cam_id: Id) -> Vec<Sample> {
    let path = colormask_path();
    let Ok(set) = load_colormask_set_or_empty(&path) else {
        return Vec::new();
    };
    let Some(stored) = set.samples(cam_id) else {
        return Vec::new();
    };
    // 디스크에는 BGR만 — 오버레이 좌표 없음
    return stored
        .iter()
        .map(|&bgr| Sample { x: -1, y: -1, bgr })
        .collect();
}

fn hint_existing(cam_id: Id, n_samples: usize) {
    let path = colormask_path();
    let Ok(set) = load_colormask_set_or_empty(&path) else {
        return;
    };
    if let Some(p) = set.params(cam_id) {
        println!(
            "existing {} cam{}: space={} c0=[{},{}] c1=[{},{}] c2=[{},{}] samples={}",
            path.display(),
            cam_id.0,
            p.space,
            p.c0_min,
            p.c0_max,
            p.c1_min,
            p.c1_max,
            p.c2_min,
            p.c2_max,
            n_samples
        );
    } else if n_samples > 0 {
        println!(
            "loaded {} samples from {} cam{} (no range yet)",
            n_samples,
            path.display(),
            cam_id.0
        );
    }
}

fn make_mask_bgr(bgr: &Mat, space: ColorSpace, range: ChannelRange) -> Result<Mat> {
    let mut converted = Mat::default();
    let code = match space {
        ColorSpace::Ycrcb => imgproc::COLOR_BGR2YCrCb,
        ColorSpace::Hsv => imgproc::COLOR_BGR2HSV,
    };
    imgproc::cvt_color(
        bgr,
        &mut converted,
        code,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    let lo = Scalar::new(
        f64::from(range.c0_min),
        f64::from(range.c1_min),
        f64::from(range.c2_min),
        0.0,
    );
    let hi = Scalar::new(
        f64::from(range.c0_max),
        f64::from(range.c1_max),
        f64::from(range.c2_max),
        0.0,
    );
    let mut mask = Mat::default();
    opencv::core::in_range(&converted, &lo, &hi, &mut mask)?;
    let mut mask_bgr = Mat::default();
    imgproc::cvt_color(
        &mask,
        &mut mask_bgr,
        imgproc::COLOR_GRAY2BGR,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    return Ok(mask_bgr);
}

fn empty_bgr_like(bgr: &Mat) -> Result<Mat> {
    return Ok(Mat::zeros(bgr.rows(), bgr.cols(), bgr.typ())?.to_mat()?);
}

fn space_axis_names(space: ColorSpace) -> [&'static str; 3] {
    return match space {
        ColorSpace::Ycrcb => ["Y", "Cr", "Cb"],
        ColorSpace::Hsv => ["H", "S", "V"],
    };
}

/// 채널 값 0..=255 → 축 픽셀. `lo_px`가 0, `hi_px`가 255.
fn map_u8_axis(v: u8, lo_px: i32, hi_px: i32) -> i32 {
    let t = f64::from(v) / 255.0;
    return (f64::from(lo_px) + (f64::from(hi_px) - f64::from(lo_px)) * t).round() as i32;
}

/// 정규화 좌표(0..=1)의 아이소메트릭 투영. y는 "위" 방향(이미지 좌표 변환 전).
fn iso_project(x: f64, y: f64, z: f64) -> (f64, f64) {
    let sx = (x - z) * 3f64.sqrt() / 2.0;
    let sy = y + (x + z) * 0.5;
    return (sx, sy);
}

fn iso_project_u8(c0: u8, c1: u8, c2: u8) -> (f64, f64) {
    return iso_project(
        f64::from(c0) / 255.0,
        f64::from(c1) / 255.0,
        f64::from(c2) / 255.0,
    );
}

fn aabb_corners(r: ChannelRange) -> [[u8; 3]; 8] {
    let a = [r.c0_min, r.c1_min, r.c2_min];
    let b = [r.c0_max, r.c1_max, r.c2_max];
    let mut out = [[0u8; 3]; 8];
    for i in 0..8 {
        out[i] = [
            if i & 1 != 0 { b[0] } else { a[0] },
            if i & 2 != 0 { b[1] } else { a[1] },
            if i & 4 != 0 { b[2] } else { a[2] },
        ];
    }
    return out;
}

fn build_swatch(width: i32, samples: &[Sample]) -> Result<Mat> {
    let w = width.max(1);
    let mut strip = Mat::zeros(SWATCH_H, w, opencv::core::CV_8UC3)?.to_mat()?;
    if samples.is_empty() {
        return Ok(strip);
    }
    let cell = (w / samples.len() as i32).max(1);
    for (i, s) in samples.iter().enumerate() {
        let x0 = i as i32 * cell;
        let x1 = if i + 1 == samples.len() {
            w
        } else {
            ((i + 1) as i32 * cell).min(w)
        };
        let color = Scalar::new(
            f64::from(s.bgr[0]),
            f64::from(s.bgr[1]),
            f64::from(s.bgr[2]),
            0.0,
        );
        imgproc::rectangle(
            &mut strip,
            Rect::new(x0, 0, (x1 - x0).max(1), SWATCH_H),
            color,
            -1,
            imgproc::LINE_8,
            0,
        )?;
    }
    return Ok(strip);
}

fn build_scatter(
    width: i32,
    height: i32,
    chs: &[[u8; 3]],
    bgrs: &[[u8; 3]],
    axis_i: usize,
    axis_j: usize,
    label: &str,
    range: Option<ChannelRange>,
) -> Result<Mat> {
    let mut panel = Mat::zeros(height, width, opencv::core::CV_8UC3)?.to_mat()?;
    // 어두운 배경
    imgproc::rectangle(
        &mut panel,
        Rect::new(0, 0, width, height),
        Scalar::new(24.0, 24.0, 24.0, 0.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    let x0 = PLOT_PAD;
    let x1 = (width - PLOT_PAD).max(x0 + 1);
    let y0 = PLOT_PAD; // top = channel max (j)
    let y1 = (height - PLOT_PAD).max(y0 + 1);

    // 축
    let axis_col = Scalar::new(80.0, 80.0, 80.0, 0.0);
    imgproc::line(
        &mut panel,
        opencv::core::Point::new(x0, y1),
        opencv::core::Point::new(x1, y1),
        axis_col,
        1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        &mut panel,
        opencv::core::Point::new(x0, y0),
        opencv::core::Point::new(x0, y1),
        axis_col,
        1,
        imgproc::LINE_8,
        0,
    )?;

    if let Some(r) = range {
        let lo = [r.c0_min, r.c1_min, r.c2_min];
        let hi = [r.c0_max, r.c1_max, r.c2_max];
        let rx0 = map_u8_axis(lo[axis_i], x0, x1);
        let rx1 = map_u8_axis(hi[axis_i], x0, x1);
        let ry_hi = map_u8_axis(hi[axis_j], y1, y0); // j max → 위
        let ry_lo = map_u8_axis(lo[axis_j], y1, y0);
        let rw = (rx1 - rx0).abs().max(1);
        let rh = (ry_lo - ry_hi).abs().max(1);
        let left = rx0.min(rx1);
        let top = ry_hi.min(ry_lo);
        imgproc::rectangle(
            &mut panel,
            Rect::new(left, top, rw, rh),
            Scalar::new(0.0, 255.0, 255.0, 0.0),
            1,
            imgproc::LINE_8,
            0,
        )?;
    }

    for (ch, bgr) in chs.iter().zip(bgrs.iter()) {
        let px = map_u8_axis(ch[axis_i], x0, x1);
        let py = map_u8_axis(ch[axis_j], y1, y0);
        imgproc::circle(
            &mut panel,
            opencv::core::Point::new(px, py),
            3,
            Scalar::new(f64::from(bgr[0]), f64::from(bgr[1]), f64::from(bgr[2]), 0.0),
            -1,
            imgproc::LINE_8,
            0,
        )?;
    }

    Preview::draw_cam_label(&mut panel, label, Scalar::new(200.0, 200.0, 200.0, 0.0))?;
    return Ok(panel);
}

fn build_iso_cube(
    width: i32,
    height: i32,
    chs: &[[u8; 3]],
    bgrs: &[[u8; 3]],
    range: Option<ChannelRange>,
) -> Result<Mat> {
    let mut panel = Mat::zeros(height, width, opencv::core::CV_8UC3)?.to_mat()?;
    imgproc::rectangle(
        &mut panel,
        Rect::new(0, 0, width, height),
        Scalar::new(24.0, 24.0, 24.0, 0.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;

    let mut pts: Vec<(f64, f64)> = Vec::new();
    if let Some(r) = range {
        for c in aabb_corners(r) {
            pts.push(iso_project_u8(c[0], c[1], c[2]));
        }
    }
    for ch in chs {
        pts.push(iso_project_u8(ch[0], ch[1], ch[2]));
    }
    // 전체 공간 모서리도 포함해 스케일 안정화
    for c in [
        [0u8, 0, 0],
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [255, 255, 255],
    ] {
        pts.push(iso_project_u8(c[0], c[1], c[2]));
    }

    let (mut min_x, mut max_x) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_y, mut max_y) = (f64::INFINITY, f64::NEG_INFINITY);
    for &(x, y) in &pts {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    if !min_x.is_finite() {
        Preview::draw_cam_label(&mut panel, "iso", Scalar::new(200.0, 200.0, 200.0, 0.0))?;
        return Ok(panel);
    }
    let dx = (max_x - min_x).max(1e-6);
    let dy = (max_y - min_y).max(1e-6);
    let pad = f64::from(PLOT_PAD);
    let usable_w = f64::from(width) - 2.0 * pad;
    let usable_h = f64::from(height) - 2.0 * pad;
    let to_px = |x: f64, y: f64| -> opencv::core::Point {
        let u = pad + (x - min_x) / dx * usable_w;
        let v = pad + (1.0 - (y - min_y) / dy) * usable_h; // y up
        return opencv::core::Point::new(u.round() as i32, v.round() as i32);
    };

    if let Some(r) = range {
        let corners = aabb_corners(r);
        let wire = Scalar::new(0.0, 255.0, 255.0, 0.0);
        for i in 0..8usize {
            for bit in 0..3usize {
                if i & (1 << bit) != 0 {
                    continue;
                }
                let j = i | (1 << bit);
                let a = corners[i];
                let b = corners[j];
                let pa = to_px(
                    iso_project_u8(a[0], a[1], a[2]).0,
                    iso_project_u8(a[0], a[1], a[2]).1,
                );
                let pb = to_px(
                    iso_project_u8(b[0], b[1], b[2]).0,
                    iso_project_u8(b[0], b[1], b[2]).1,
                );
                imgproc::line(&mut panel, pa, pb, wire, 1, imgproc::LINE_8, 0)?;
            }
        }
    }

    for (ch, bgr) in chs.iter().zip(bgrs.iter()) {
        let (sx, sy) = iso_project_u8(ch[0], ch[1], ch[2]);
        let p = to_px(sx, sy);
        imgproc::circle(
            &mut panel,
            p,
            3,
            Scalar::new(f64::from(bgr[0]), f64::from(bgr[1]), f64::from(bgr[2]), 0.0),
            -1,
            imgproc::LINE_8,
            0,
        )?;
    }

    Preview::draw_cam_label(
        &mut panel,
        "iso AABB",
        Scalar::new(200.0, 200.0, 200.0, 0.0),
    )?;
    return Ok(panel);
}

fn build_range_viz(
    width: i32,
    samples: &[Sample],
    space: ColorSpace,
    range: Option<ChannelRange>,
) -> Result<Mat> {
    let w = width.max(1);
    let swatch = build_swatch(w, samples)?;

    let mut chs = Vec::with_capacity(samples.len());
    let mut bgrs = Vec::with_capacity(samples.len());
    for s in samples {
        chs.push(bgr_to_space(s.bgr, space)?);
        bgrs.push(s.bgr);
    }

    let names = space_axis_names(space);
    let cell = (w / 4).max(1);
    let h = VIZ_H;
    let p01 = build_scatter(
        cell,
        h,
        &chs,
        &bgrs,
        0,
        1,
        &format!("{}-{}", names[0], names[1]),
        range,
    )?;
    let p02 = build_scatter(
        cell,
        h,
        &chs,
        &bgrs,
        0,
        2,
        &format!("{}-{}", names[0], names[2]),
        range,
    )?;
    let p12 = build_scatter(
        cell,
        h,
        &chs,
        &bgrs,
        1,
        2,
        &format!("{}-{}", names[1], names[2]),
        range,
    )?;
    let iso = build_iso_cube(w - cell * 3, h, &chs, &bgrs, range)?;
    let row = Preview::hstack_bgr(&[p01, p02, p12, iso])?;
    return vstack_bgr(&swatch, &row);
}

fn vstack_bgr(top: &Mat, bottom: &Mat) -> Result<Mat> {
    let w = top.cols().max(bottom.cols()).max(1);
    let pad = |m: &Mat| -> Result<Mat> {
        if m.cols() == w {
            return Ok(m.try_clone()?);
        }
        let mut canvas = Mat::zeros(m.rows(), w, m.typ())?.to_mat()?;
        let roi = Rect::new(0, 0, m.cols(), m.rows());
        let mut dst = Mat::roi_mut(&mut canvas, roi)?;
        m.copy_to(&mut dst)?;
        return Ok(canvas);
    };
    let a = pad(top)?;
    let b = pad(bottom)?;
    let mut out = Mat::default();
    opencv::core::vconcat(&Vector::<Mat>::from_iter([a, b]), &mut out)?;
    return Ok(out);
}

fn main() -> Result<()> {
    let args = Args::parse();
    let margin = args.margin.min(32);
    let trim_pct = args.trim.clamp(0.0, 49.0);
    let cam_id = args.cam.camera_id().map_err(anyhow::Error::msg)?;
    let mut source = open_source(&args)?;
    let mut space = args.space;
    let wait_ms = args
        .wait_ms
        .unwrap_or(if args.offline.has_offline() || args.images.is_some() {
            33
        } else {
            1
        });

    let window = "tune:colormask";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;

    let mouse: Arc<Mutex<PixelPickMouse>> = Arc::new(Mutex::new(PixelPickMouse::default()));
    {
        let mouse = Arc::clone(&mouse);
        highgui::set_mouse_callback(
            window,
            Some(Box::new(move |event, x, y, flags| {
                if let Ok(mut m) = mouse.lock() {
                    m.on_event(event, x, y, flags);
                }
            })),
        )?;
    }

    let mut samples: Vec<Sample> = load_samples_for_cam(cam_id);
    let mut frozen = false;
    let mut freeze_img: Option<Mat> = None;
    let mut n = 0usize;
    let mut display_scale = 1.0;

    // 저장된 space가 있으면 시작 space로 맞춤
    if let Ok(set) = load_colormask_set_or_empty(&colormask_path()) {
        if let Some(p) = set.params(cam_id) {
            space = p.space;
        }
    }

    println!(
        "tune-colormask cam={} space={space} margin={margin} trim={trim_pct}% → {}",
        cam_id.0,
        colormask_path().display()
    );
    hint_existing(cam_id, samples.len());
    if !samples.is_empty() {
        println!(
            "resumed {} samples — pick more or p to re-save",
            samples.len()
        );
    }
    println!(
        "LMB/Enter=pick  arrows|hjkl=1px  Shift+move=loupe  z=undo  c=clear  Space=freeze  s=space  p=save→{}  q=quit",
        colormask_path().display()
    );

    loop {
        let frame_img = if frozen {
            match &freeze_img {
                Some(img) => img.try_clone().map_err(|e| anyhow::anyhow!("clone: {e}"))?,
                None => {
                    let Some(frame) = source.next_frame() else {
                        break;
                    };
                    frame
                        .image
                        .try_clone()
                        .map_err(|e| anyhow::anyhow!("clone: {e}"))?
                }
            }
        } else {
            let Some(frame) = source.next_frame() else {
                break;
            };
            let img = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            freeze_img = Some(img.try_clone().map_err(|e| anyhow::anyhow!("clone: {e}"))?);
            img
        };

        let panel_w = frame_img.cols();
        let panel_h = frame_img.rows();

        // drain clicks → sample on original panel only; Shift-hover for loupe
        let (clicks, hover) = {
            let mut m = mouse.lock().expect("mouse lock");
            m.sync(display_scale, panel_w, panel_h);
            let clicks = m.drain_clicks();
            (clicks, m.hover)
        };
        for (mx, my) in clicks {
            if mx < 0 || my < 0 || mx >= panel_w || my >= panel_h {
                continue;
            }
            if let Some(bgr) = read_bgr_avg(&frame_img, mx, my, SAMPLE_RADIUS) {
                samples.push(Sample { x: mx, y: my, bgr });
                println!(
                    "sample #{} px=({mx},{my}) BGR=[{},{},{}]",
                    samples.len(),
                    bgr[0],
                    bgr[1],
                    bgr[2]
                );
            }
        }

        let (range_y, range_h) = ranges_from_samples(&samples, margin, trim_pct)?;
        let active_range = match space {
            ColorSpace::Ycrcb => range_y,
            ColorSpace::Hsv => range_h,
        };

        let mut original = frame_img
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
        for (i, s) in samples.iter().enumerate() {
            if s.x < 0 || s.y < 0 {
                continue; // resumed BGR-only — no pixel overlay
            }
            let color = if i + 1 == samples.len() {
                Scalar::new(0.0, 255.0, 0.0, 0.0)
            } else {
                Scalar::new(0.0, 200.0, 255.0, 0.0)
            };
            Preview::draw_circle_px(
                &mut original,
                Pixel::new(f64::from(s.x), f64::from(s.y)),
                6,
                color,
                2,
            )?;
        }
        if frozen {
            Preview::draw_cam_label(&mut original, "FROZEN", Scalar::new(0.0, 0.0, 255.0, 0.0))?;
        }
        Preview::draw_cam_label(
            &mut original,
            "original",
            Scalar::new(255.0, 255.0, 255.0, 0.0),
        )?;

        let mut mask = match active_range {
            Some(r) => make_mask_bgr(&frame_img, space, r)?,
            None => empty_bgr_like(&frame_img)?,
        };
        Preview::draw_cam_label(&mut mask, "mask", Scalar::new(0.0, 255.0, 255.0, 0.0))?;

        let top = Preview::hstack_bgr(&[original, mask])?;
        let strip = build_range_viz(top.cols(), &samples, space, active_range)?;
        let mut mosaic = vstack_bgr(&top, &strip)?;

        let range_txt = match active_range {
            Some(r) => format!(
                "[{},{}] [{},{}] [{},{}]",
                r.c0_min, r.c0_max, r.c1_min, r.c1_max, r.c2_min, r.c2_max
            ),
            None => "no samples".into(),
        };
        let lines = [
            format!("tune  space={space}  samples={}", samples.len()),
            format!("{}  margin={margin}  trim={trim_pct}%", range_txt),
            space_label(space).to_string(),
        ];
        Preview::draw_debug_lines(&mut mosaic, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        Preview::draw_help_lines(
            &mut mosaic,
            &[
                "LMB/Enter pick",
                "arrows|hjkl 1px  Shift loupe",
                "z undo  c clear",
                "Space freeze",
                "s ycrcb|hsv",
                "p save→data/colormask.json",
                "q/ESC quit",
            ],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;
        if let Some((hx, hy)) = hover {
            if hx >= 0 && hy >= 0 && hx < panel_w && hy < panel_h {
                let _ = Preview::draw_pixel_loupe(&mut mosaic, &frame_img, hx, hy);
            }
        }

        let shown = Preview::show_bgr(window, &mosaic, wait_ms)?;
        display_scale = shown.scale;
        match shown.action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(k) => {
                if let Some((dx, dy)) = Preview::arrow_delta(k) {
                    let mut m = mouse.lock().expect("mouse lock");
                    m.sync(display_scale, panel_w, panel_h);
                    m.nudge(dx, dy, panel_w, panel_h);
                    continue;
                }
                if k == 13 || k == 10 {
                    mouse.lock().expect("mouse lock").confirm();
                    continue;
                }
                let key = k & 0xff;
                if key == i32::from(b' ') {
                    frozen = !frozen;
                    println!("{}", if frozen { "frozen" } else { "live" });
                } else if key == i32::from(b's') || key == i32::from(b'S') {
                    space = match space {
                        ColorSpace::Ycrcb => ColorSpace::Hsv,
                        ColorSpace::Hsv => ColorSpace::Ycrcb,
                    };
                    println!("space={space}");
                } else if key == i32::from(b'z') || key == i32::from(b'Z') || key == 8 {
                    if samples.pop().is_some() {
                        println!("undo → {} samples", samples.len());
                    }
                } else if key == i32::from(b'c') || key == i32::from(b'C') {
                    samples.clear();
                    println!("cleared");
                } else if key == i32::from(b'p') || key == i32::from(b'P') {
                    let (y, h) = ranges_from_samples(&samples, margin, trim_pct)?;
                    let active = match space {
                        ColorSpace::Ycrcb => y,
                        ColorSpace::Hsv => h,
                    };
                    if let Some(r) = active {
                        upsert_colormask(cam_id, space, r, &samples)?;
                    } else {
                        println!("(save skipped: need samples)");
                    }
                }
            }
        }

        n += 1;
        if args.max_frames > 0 && n >= args.max_frames {
            break;
        }
    }

    // 종료 시 저장
    if !samples.is_empty() {
        let (y, h) = ranges_from_samples(&samples, margin, trim_pct)?;
        let active = match space {
            ColorSpace::Ycrcb => y,
            ColorSpace::Hsv => h,
        };
        if let Some(r) = active {
            upsert_colormask(cam_id, space, r, &samples)?;
        }
    }

    Preview::destroy_window(window);
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_range_margin_trim0_is_minmax() {
        let chs = [[100u8, 150, 80], [110, 160, 90]];
        let r = ChannelRange::from_channels(&chs, 3, 0.0).unwrap();
        assert_eq!(r.c0_min, 97);
        assert_eq!(r.c0_max, 113);
        assert_eq!(r.c1_min, 147);
        assert_eq!(r.c1_max, 163);
        assert_eq!(r.c2_min, 77);
        assert_eq!(r.c2_max, 93);
    }

    #[test]
    fn percentile_ignores_highlight_and_green_outliers() {
        // 주황 본체(H≈18) 다수 + 하이라이트(S↓) + 초록끼(H↑) 각 1개
        let mut chs = vec![[18u8, 180, 200]; 18];
        chs.push([20, 20, 250]); // highlight: S 붕괴
        chs.push([50, 160, 190]); // green fringe: H 팽창
        let r = ChannelRange::from_channels(&chs, 0, 10.0).unwrap();
        assert!(
            r.c0_max < 50,
            "H max must reject green fringe, got {}",
            r.c0_max
        );
        assert!(
            r.c1_min > 20,
            "S min must reject highlight, got {}",
            r.c1_min
        );
        // 본체 클러스터 근처로 수축 (보간으로 정확히 모드값일 필요는 없음)
        assert!(r.c0_min <= 20 && r.c0_max <= 25);
        assert!(r.c1_min >= 160);
    }

    #[test]
    fn percentile_empty_returns_none() {
        assert!(ChannelRange::from_channels(&[], 0, 10.0).is_none());
    }

    #[test]
    fn map_u8_axis_endpoints() {
        assert_eq!(map_u8_axis(0, 10, 110), 10);
        assert_eq!(map_u8_axis(255, 10, 110), 110);
        assert_eq!(map_u8_axis(127, 0, 254), 127);
    }

    #[test]
    fn iso_project_separates_axes() {
        let o = iso_project(0.0, 0.0, 0.0);
        let x = iso_project(1.0, 0.0, 0.0);
        let y = iso_project(0.0, 1.0, 0.0);
        let z = iso_project(0.0, 0.0, 1.0);
        assert!((o.0 - 0.0).abs() < 1e-9 && (o.1 - 0.0).abs() < 1e-9);
        assert!(x.0 > 0.0);
        assert!(z.0 < 0.0);
        assert!(y.1 > o.1);
        // x와 z는 수평으로 반대, y는 주로 수직
        assert!((x.0 + z.0).abs() < 1e-9);
    }

    #[test]
    fn aabb_corners_are_extremes() {
        let r = ChannelRange {
            c0_min: 10,
            c0_max: 20,
            c1_min: 30,
            c1_max: 40,
            c2_min: 50,
            c2_max: 60,
        };
        let cs = aabb_corners(r);
        assert_eq!(cs.len(), 8);
        assert!(cs.contains(&[10, 30, 50]));
        assert!(cs.contains(&[20, 40, 60]));
    }
}
