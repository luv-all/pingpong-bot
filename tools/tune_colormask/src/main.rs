//! 탁구공 색 범위 튜닝 — 픽커 → min/max → dry-run Rust 출력.
//!
//! 레이아웃: (original | mask) / 색상 띠. 파일 저장 없음.

mod cli;

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use clap::Parser;
use opencv::core::{Rect, Scalar, Vec3b, Vector};
use opencv::imgproc;
use opencv::prelude::*;
use opencv::highgui;
use pingpong_bot::{
    CameraId, ColorSpace, ColormaskParams, FrameSource, ImageDirSource, OpenCvCapture, PixelPoint,
    PreviewAction, destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines,
    draw_help_lines, hstack_bgr, show_bgr,
};

use cli::Args;

const STRIP_H: i32 = 72;
const SAMPLE_RADIUS: i32 = 2;

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

impl ChannelRange {
    fn from_channels(chs: &[[u8; 3]], margin: u8) -> Option<Self> {
        if chs.is_empty() {
            return None;
        }
        let mut lo = [255u8; 3];
        let mut hi = [0u8; 3];
        for c in chs {
            for i in 0..3 {
                lo[i] = lo[i].min(c[i]);
                hi[i] = hi[i].max(c[i]);
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
    if let Some(images) = &args.images {
        return Ok(Box::new(
            ImageDirSource::open(CameraId(0), images)
                .map_err(anyhow::Error::msg)
                .context("images")?,
        ));
    }
    if let Some(path) = &args.path {
        return Ok(Box::new(
            OpenCvCapture::from_path(CameraId(0), path)
                .map_err(anyhow::Error::msg)
                .context("path")?,
        ));
    }
    let device = args.device.unwrap_or(0);
    return Ok(Box::new(
        OpenCvCapture::from_device(CameraId(0), device)
            .map_err(anyhow::Error::msg)
            .with_context(|| format!("device {device}"))?,
    ));
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
        Scalar::new(
            f64::from(bgr[0]),
            f64::from(bgr[1]),
            f64::from(bgr[2]),
            0.0,
        ),
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

fn ranges_from_samples(samples: &[Sample], margin: u8) -> Result<(Option<ChannelRange>, Option<ChannelRange>)> {
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
        ChannelRange::from_channels(&ycrcb, margin),
        ChannelRange::from_channels(&hsv, margin),
    ));
}

fn space_label(space: ColorSpace) -> &'static str {
    return match space {
        ColorSpace::Ycrcb => "Y/Cr/Cb",
        ColorSpace::Hsv => "H/S/V",
    };
}

fn print_params(space: ColorSpace, range: ChannelRange) {
    let p = range.to_params(space);
    let axes = space_label(space);
    println!("// paste into defaults::colormask() — space={space} ({axes})");
    println!("ColormaskParams {{");
    println!("    space: ColorSpace::{},", match space {
        ColorSpace::Ycrcb => "Ycrcb",
        ColorSpace::Hsv => "Hsv",
    });
    println!("    c0_min: {}, // {}", p.c0_min, axes.split('/').next().unwrap_or("c0"));
    println!("    c0_max: {},", p.c0_max);
    println!("    c1_min: {}, // {}", p.c1_min, axes.split('/').nth(1).unwrap_or("c1"));
    println!("    c1_max: {},", p.c1_max);
    println!("    c2_min: {}, // {}", p.c2_min, axes.split('/').nth(2).unwrap_or("c2"));
    println!("    c2_max: {},", p.c2_max);
    println!("}}");
}

fn print_all(ycrcb: Option<ChannelRange>, hsv: Option<ChannelRange>, n: usize, margin: u8) {
    println!("--- tune-colormask samples={n} margin={margin} ---");
    match ycrcb {
        Some(r) => print_params(ColorSpace::Ycrcb, r),
        None => println!("(ycrcb: need samples)"),
    }
    println!();
    match hsv {
        Some(r) => print_params(ColorSpace::Hsv, r),
        None => println!("(hsv: need samples)"),
    }
    println!("----------------------------------------------");
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

fn space_to_bgr_pixel(c0: u8, c1: u8, c2: u8, space: ColorSpace) -> Result<[u8; 3]> {
    let pixel = Mat::new_rows_cols_with_default(1, 1, opencv::core::CV_8UC3, Scalar::new(
        f64::from(c0),
        f64::from(c1),
        f64::from(c2),
        0.0,
    ))?;
    let mut out = Mat::default();
    let code = match space {
        ColorSpace::Ycrcb => imgproc::COLOR_YCrCb2BGR,
        ColorSpace::Hsv => imgproc::COLOR_HSV2BGR,
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

fn build_strip(width: i32, samples: &[Sample], space: ColorSpace, range: Option<ChannelRange>) -> Result<Mat> {
    let w = width.max(1);
    let mut strip = Mat::zeros(STRIP_H, w, opencv::core::CV_8UC3)?.to_mat()?;

    // 상단: 샘플 swatch
    let swatch_h = STRIP_H / 2;
    if samples.is_empty() {
        // keep black
    } else {
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
                Rect::new(x0, 0, (x1 - x0).max(1), swatch_h),
                color,
                -1,
                imgproc::LINE_8,
                0,
            )?;
        }
    }

    // 하단: min→max 대각 보간 띠 (현재 space)
    if let Some(r) = range {
        for x in 0..w {
            let t = if w <= 1 {
                0.0
            } else {
                f64::from(x) / f64::from(w - 1)
            };
            let lerp = |a: u8, b: u8| -> u8 {
                return (f64::from(a) + (f64::from(b) - f64::from(a)) * t).round() as u8;
            };
            let c0 = lerp(r.c0_min, r.c0_max);
            let c1 = lerp(r.c1_min, r.c1_max);
            let c2 = lerp(r.c2_min, r.c2_max);
            let bgr = space_to_bgr_pixel(c0, c1, c2, space).unwrap_or([0, 0, 0]);
            for y in swatch_h..STRIP_H {
                let px: &mut Vec3b = strip.at_2d_mut(y, x)?;
                *px = Vec3b::from(bgr);
            }
        }
    }

    return Ok(strip);
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
    let mut source = open_source(&args)?;
    let mut space = args.space;
    let wait_ms = args.wait_ms.unwrap_or(if args.path.is_some() || args.images.is_some() {
        33
    } else {
        1
    });

    let window = "tune:colormask";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;

    let pending: Arc<Mutex<Vec<(i32, i32)>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let pending = Arc::clone(&pending);
        highgui::set_mouse_callback(
            window,
            Some(Box::new(move |event, x, y, _flags| {
                if event == highgui::EVENT_LBUTTONDOWN {
                    if let Ok(mut q) = pending.lock() {
                        q.push((x, y));
                    }
                }
            })),
        )?;
    }

    let mut samples: Vec<Sample> = Vec::new();
    let mut frozen = false;
    let mut freeze_img: Option<Mat> = None;
    let mut n = 0usize;

    println!(
        "tune-colormask space={space} margin={margin}  LMB=pick  z=undo  c=clear  Space=freeze  s=space  p=print  q=quit"
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
            freeze_img = Some(
                img.try_clone()
                    .map_err(|e| anyhow::anyhow!("clone: {e}"))?,
            );
            img
        };

        let panel_w = frame_img.cols();
        let panel_h = frame_img.rows();

        // drain clicks → sample on original panel only
        let clicks: Vec<(i32, i32)> = {
            let mut q = pending.lock().expect("pending lock");
            let out = q.clone();
            q.clear();
            out
        };
        for (mx, my) in clicks {
            if mx < 0 || my < 0 || mx >= panel_w || my >= panel_h {
                continue;
            }
            if let Some(bgr) = read_bgr_avg(&frame_img, mx, my, SAMPLE_RADIUS) {
                samples.push(Sample {
                    x: mx,
                    y: my,
                    bgr,
                });
                println!(
                    "sample #{} px=({mx},{my}) BGR=[{},{},{}]",
                    samples.len(),
                    bgr[0],
                    bgr[1],
                    bgr[2]
                );
            }
        }

        let (range_y, range_h) = ranges_from_samples(&samples, margin)?;
        let active_range = match space {
            ColorSpace::Ycrcb => range_y,
            ColorSpace::Hsv => range_h,
        };

        let mut original = frame_img
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
        for (i, s) in samples.iter().enumerate() {
            let color = if i + 1 == samples.len() {
                Scalar::new(0.0, 255.0, 0.0, 0.0)
            } else {
                Scalar::new(0.0, 200.0, 255.0, 0.0)
            };
            draw_circle_px(
                &mut original,
                PixelPoint::new(f64::from(s.x), f64::from(s.y)),
                6,
                color,
                2,
            )?;
        }
        if frozen {
            draw_cam_label(&mut original, "FROZEN", Scalar::new(0.0, 0.0, 255.0, 0.0))?;
        }
        draw_cam_label(
            &mut original,
            "original",
            Scalar::new(255.0, 255.0, 255.0, 0.0),
        )?;

        let mut mask = match active_range {
            Some(r) => make_mask_bgr(&frame_img, space, r)?,
            None => empty_bgr_like(&frame_img)?,
        };
        draw_cam_label(&mut mask, "mask", Scalar::new(0.0, 255.0, 255.0, 0.0))?;

        let top = hstack_bgr(&[original, mask])?;
        let strip = build_strip(top.cols(), &samples, space, active_range)?;
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
            format!("{}  margin={margin}", range_txt),
            space_label(space).to_string(),
        ];
        draw_debug_lines(&mut mosaic, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        draw_help_lines(
            &mut mosaic,
            &[
                "LMB pick",
                "z undo  c clear",
                "Space freeze",
                "s ycrcb|hsv",
                "p print",
                "q/ESC quit",
            ],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;

        match show_bgr(window, &mosaic, wait_ms)? {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(key) if key == i32::from(b' ') => {
                frozen = !frozen;
                println!("{}", if frozen { "frozen" } else { "live" });
            }
            PreviewAction::Key(key) if key == i32::from(b's') || key == i32::from(b'S') => {
                space = match space {
                    ColorSpace::Ycrcb => ColorSpace::Hsv,
                    ColorSpace::Hsv => ColorSpace::Ycrcb,
                };
                println!("space={space}");
            }
            PreviewAction::Key(key) if key == i32::from(b'z') || key == i32::from(b'Z') || key == 8 => {
                if samples.pop().is_some() {
                    println!("undo → {} samples", samples.len());
                }
            }
            PreviewAction::Key(key) if key == i32::from(b'c') || key == i32::from(b'C') => {
                samples.clear();
                println!("cleared");
            }
            PreviewAction::Key(key) if key == i32::from(b'p') || key == i32::from(b'P') => {
                let (y, h) = ranges_from_samples(&samples, margin)?;
                print_all(y, h, samples.len(), margin);
            }
            PreviewAction::Key(_) => {}
        }

        n += 1;
        if args.max_frames > 0 && n >= args.max_frames {
            break;
        }
    }

    // 종료 시 한 번 더 출력
    if !samples.is_empty() {
        let (y, h) = ranges_from_samples(&samples, margin)?;
        print_all(y, h, samples.len(), margin);
    }

    destroy_window(window);
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_range_margin() {
        let chs = [[100u8, 150, 80], [110, 160, 90]];
        let r = ChannelRange::from_channels(&chs, 3).unwrap();
        assert_eq!(r.c0_min, 97);
        assert_eq!(r.c0_max, 113);
        assert_eq!(r.c1_min, 147);
        assert_eq!(r.c1_max, 163);
        assert_eq!(r.c2_min, 77);
        assert_eq!(r.c2_max, 93);
    }
}
