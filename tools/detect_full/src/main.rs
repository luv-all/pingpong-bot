//! fuse 본선 디버그 — `defaults::detector()` + ROI 토글 (`r`).
//!
//! appearance만: [detect-appearance](../detect_appearance/README.md).

mod cli;

use anyhow::{Context, Result};
use clap::Parser;
use opencv::core::Scalar;
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::{
    BallDetector, CameraId, FrameSource, ImageDirSource, OpenCvCapture, PixelPoint, PreviewAction,
    destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines, draw_help_lines, hstack_bgr,
    show_bgr,
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
    println!("{} (from defaults::detector())", detector);
    println!("keys: r = ROI toggle, q/ESC = quit");

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
        let mut original = frame
            .image
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
        let mut side = Mat::zeros(original.rows(), original.cols(), original.typ())?.to_mat()?;

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
                &mut side,
                r,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                2,
                imgproc::LINE_8,
                0,
            )?;
        }

        if let Some(p) = pixel {
            hits += 1;
            let mode = if detector.used_roi { "roi" } else { "full" };
            println!("frame={n} {mode} px=({:.1}, {:.1})", p.x, p.y);
            draw_circle_px(&mut original, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            draw_circle_px(&mut side, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
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

        draw_cam_label(
            &mut original,
            "original",
            Scalar::new(255.0, 255.0, 255.0, 0.0),
        )?;
        let side_label = if detector.used_roi {
            "roi"
        } else if detector.roi_enabled {
            "acquire"
        } else {
            "roi-off"
        };
        draw_cam_label(&mut side, side_label, Scalar::new(0.0, 255.0, 255.0, 0.0))?;

        let mut mosaic = hstack_bgr(&[original, side])?;
        let hit_rate = if n == 0 {
            0.0
        } else {
            100.0 * hits as f64 / (n + 1) as f64
        };
        let lines = [
            detector.to_string(),
            match pixel {
                Some(p) => format!(
                    "{}  px=({:.1},{:.1})",
                    if detector.used_roi { "roi" } else { "full" },
                    p.x,
                    p.y
                ),
                None => "miss".to_string(),
            },
            format!("hits={hits}/{}  ({hit_rate:.0}%)", n + 1),
        ];
        draw_debug_lines(&mut mosaic, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        draw_help_lines(
            &mut mosaic,
            &["r ROI toggle", "q/ESC quit"],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;

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
                PreviewAction::Continue | PreviewAction::Key(_) => {}
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
    return Ok(());
}
