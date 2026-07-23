//! fuse 본선 디버그 — adaptive ROI + 누적 파이프라인 패널.
//!
//! 스텝: `0 original → 1 colormask → 2 +contour → 3 roi`
//! track 중이면 1·2는 ROI 크롭에서만 계산(본선과 동일).
//! 키: `r` ROI · `[` `]` k · `,` `.` m · `-` `=` pad · `p` paste · `q`/ESC

mod cli;

use anyhow::{Context, Result};
use clap::Parser;
use opencv::core::{Rect, Scalar, Vector};
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::{
    BallDetector, CameraId, ColorContourCascade, Frame, FrameSource, ImageDirSource, OpenCvCapture,
    PixelPoint, PreviewAction, RoiTrack, Scorer, destroy_window, draw_cam_label, draw_circle_px,
    draw_debug_lines, draw_help_lines, hstack_bgr, show_bgr,
};

use cli::Args;

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

fn empty_like(frame: &Frame) -> Result<Mat> {
    return Ok(Mat::zeros(frame.image.rows(), frame.image.cols(), frame.image.typ())?.to_mat()?);
}

fn paste_at(dst: &mut Mat, src: &Mat, r: Rect) -> Result<()> {
    if src.cols() != r.width || src.rows() != r.height {
        return Ok(());
    }
    let mut view = Mat::roi_mut(dst, r)?;
    src.copy_to(&mut view)?;
    return Ok(());
}

/// 본선과 같은 영역에서 cascade 스텝을 돌린다. ROI track이면 크롭.
fn cascade_steps(
    cascade: &mut ColorContourCascade,
    scorer: &Scorer,
    frame: &Frame,
    roi: Option<Rect>,
) -> Result<(Option<PixelPoint>, Mat, Mat)> {
    let Some(r) = roi else {
        let (px, cm, cas) = cascade.detect_debug(frame, scorer);
        return Ok((px, cm, cas));
    };

    let view = Mat::roi(&frame.image, r).map_err(|e| anyhow::anyhow!("roi view: {e}"))?;
    let owned = view
        .try_clone()
        .map_err(|e| anyhow::anyhow!("roi clone: {e}"))?;
    let local = Frame {
        camera_id: frame.camera_id,
        image: owned,
        timestamp: frame.timestamp,
    };
    let (local_px, cm_local, cas_local) = cascade.detect_debug(&local, scorer);

    let mut cm_full = empty_like(frame)?;
    let mut cas_full = empty_like(frame)?;
    paste_at(&mut cm_full, &cm_local, r)?;
    paste_at(&mut cas_full, &cas_local, r)?;

    let px = local_px.map(|p| PixelPoint::new(p.x + f64::from(r.x), p.y + f64::from(r.y)));
    return Ok((px, cm_full, cas_full));
}

fn handle_tune_key(detector: &mut RoiTrack, key: i32) -> bool {
    let p = &mut detector.params;
    let handled = match key {
        k if k == i32::from(b'[') => {
            p.k = (p.k - 0.25).max(0.0);
            true
        }
        k if k == i32::from(b']') => {
            p.k += 0.25;
            true
        }
        k if k == i32::from(b',') => {
            p.m = (p.m - 0.25).max(0.0);
            true
        }
        k if k == i32::from(b'.') => {
            p.m += 0.25;
            true
        }
        k if k == i32::from(b'-') => {
            p.pad = (p.pad - 4).max(0);
            true
        }
        k if k == i32::from(b'=') => {
            p.pad += 4;
            true
        }
        k if k == i32::from(b'p') || k == i32::from(b'P') => {
            println!("// paste into defaults::roi()\n{}", p.to_defaults_snippet());
            false
        }
        _ => false,
    };
    if handled {
        detector.recompute_half();
        println!("{detector}");
    }
    return handled;
}

fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(dir) = &args.output {
        std::fs::create_dir_all(dir).ok();
    }

    let mut source = open_source(&args)?;
    let mut detector = pingpong_bot::detector();
    if args.no_roi {
        detector.set_roi_enabled(false);
    }

    let scorer_params = pingpong_bot::scorer();
    let scorer = Scorer::from(&scorer_params);
    let mut cascade = ColorContourCascade::new(pingpong_bot::colormask(), &scorer_params);

    println!("{detector} (defaults: colormask → contour → ROI)");
    println!("keys: r ROI  [ ] k  , . m  - = pad  p paste  q/ESC quit");

    let window = "detect:full";
    let wait_ms = args
        .wait_ms
        .unwrap_or(if args.path.is_some() || args.images.is_some() {
            33
        } else {
            1
        });
    let preview = !args.no_preview;

    let mut n = 0usize;
    let mut hits = 0usize;
    let mut last_pixel: Option<PixelPoint> = None;
    let mut prev_pixel: Option<PixelPoint> = None;

    while let Some(frame) = source.next_frame() {
        let pixel = detector.detect(&frame);

        // 본선이 이번 프레임에 쓴 영역과 동일하게 1·2 스텝을 돌린다.
        let step_roi = if detector.used_roi {
            detector.last_roi
        } else {
            None
        };
        let (step_px, mut cm_panel, mut ct_panel) =
            cascade_steps(&mut cascade, &scorer, &frame, step_roi)?;

        let mut original = frame
            .image
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;

        if let Some(r) = detector.last_roi {
            imgproc::rectangle(
                &mut original,
                r,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                2,
                imgproc::LINE_8,
                0,
            )?;
            imgproc::rectangle(
                &mut cm_panel,
                r,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                1,
                imgproc::LINE_8,
                0,
            )?;
            imgproc::rectangle(
                &mut ct_panel,
                r,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                1,
                imgproc::LINE_8,
                0,
            )?;
        }

        if let Some(p) = pixel {
            hits += 1;
            let mode = if detector.used_roi { "roi" } else { "full" };
            println!(
                "frame={n} {mode} half={} px=({:.1}, {:.1})",
                detector.half_px, p.x, p.y
            );
            draw_circle_px(&mut original, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            if let Some(prev) = prev_pixel {
                draw_circle_px(
                    &mut original,
                    prev,
                    6,
                    Scalar::new(0.0, 200.0, 255.0, 0.0),
                    1,
                )?;
            }
            prev_pixel = last_pixel;
            last_pixel = Some(p);
        } else {
            println!("frame={n} miss");
        }

        if let Some(p) = step_px.or(pixel) {
            draw_circle_px(&mut cm_panel, p, 8, Scalar::new(0.0, 255.0, 0.0, 0.0), 1)?;
            draw_circle_px(&mut ct_panel, p, 8, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
        }

        let mut roi_panel = empty_like(&frame)?;
        if let Some(r) = detector.last_roi {
            if let Ok(view) = Mat::roi(&frame.image, r) {
                if let Ok(owned) = view.try_clone() {
                    paste_at(&mut roi_panel, &owned, r)?;
                }
            }
            imgproc::rectangle(
                &mut roi_panel,
                r,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                2,
                imgproc::LINE_8,
                0,
            )?;
            if let Some(p) = pixel {
                draw_circle_px(&mut roi_panel, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            }
        } else if let Some(p) = pixel {
            original.copy_to(&mut roi_panel)?;
            draw_circle_px(&mut roi_panel, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
        }

        let roi_label = if detector.used_roi {
            "3 roi"
        } else if detector.roi_enabled {
            "3 acquire"
        } else {
            "3 roi-off"
        };

        draw_cam_label(&mut original, "0 original", Scalar::new(255.0, 255.0, 255.0, 0.0))?;
        draw_cam_label(&mut cm_panel, "1 colormask", Scalar::new(0.0, 255.0, 0.0, 0.0))?;
        draw_cam_label(
            &mut ct_panel,
            "2 +contour",
            Scalar::new(255.0, 128.0, 0.0, 0.0),
        )?;
        draw_cam_label(&mut roi_panel, roi_label, Scalar::new(0.0, 255.0, 255.0, 0.0))?;

        let hit_rate = if n == 0 {
            0.0
        } else {
            100.0 * hits as f64 / (n + 1) as f64
        };
        let r_eq = detector
            .last_area()
            .map(|a| (a / std::f64::consts::PI).sqrt())
            .unwrap_or(0.0);
        // Hershey = ASCII only. 모자이크가 아니라 패널에 그려 스케일이 폭주하지 않게.
        let lines = [
            detector.to_string(),
            match pixel {
                Some(p) => format!(
                    "{}  px=({:.1},{:.1})  r~{:.0}  cm->ct->roi",
                    if detector.used_roi { "roi" } else { "full" },
                    p.x,
                    p.y,
                    r_eq
                ),
                None => "miss".to_string(),
            },
            format!("hits={hits}/{}  ({hit_rate:.0}%)", n + 1),
        ];
        draw_debug_lines(&mut original, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        draw_help_lines(
            &mut original,
            &["r ROI | [/] k | ,/. m | -/= pad | p paste | q quit"],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;

        // 읽는 순서 = 파이프라인: 0->1 / 2->3
        let top = hstack_bgr(&[original, cm_panel])?;
        let bottom = hstack_bgr(&[ct_panel, roi_panel])?;
        let mosaic = vstack_bgr(&top, &bottom)?;

        if let Some(dir) = &args.output {
            let out = dir.join(format!("full_{n:04}.png"));
            imgcodecs::imwrite(
                out.to_str().context("out path")?,
                &mosaic,
                &opencv::core::Vector::new(),
            )?;
        }

        if preview {
            match show_bgr(window, &mosaic, wait_ms)? {
                PreviewAction::Quit => break,
                PreviewAction::Key(key) if key == i32::from(b'r') || key == i32::from(b'R') => {
                    detector.set_roi_enabled(!detector.roi_enabled);
                    println!(
                        "roi → {}",
                        if detector.roi_enabled { "on" } else { "off" }
                    );
                }
                PreviewAction::Key(key) => {
                    handle_tune_key(&mut detector, key);
                }
                PreviewAction::Continue => {}
            }
        }

        n += 1;
        if args.images.is_none() && n >= args.max_frames {
            break;
        }
    }

    if preview {
        destroy_window(window);
    }
    println!("done frames={n} hits={hits} {detector}");
    println!(
        "// paste into defaults::roi()\n{}",
        detector.params.to_defaults_snippet()
    );
    return Ok(());
}
