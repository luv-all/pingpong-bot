use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use opencv::core::Scalar;
use opencv::imgcodecs;
use opencv::prelude::*;
use pingpong_bot::{
    BallDetector, CameraId, FrameSource, ImageDirSource, OpenCvCapture, PixelPoint, PreviewAction,
    destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines, show_bgr,
};

#[derive(Parser, Debug)]
#[command(about = "공 검출 실험")]
pub struct DetectArgs {
    #[arg(long)]
    pub images: Option<PathBuf>,
    #[arg(long)]
    pub device: Option<i32>,
    #[arg(long)]
    pub path: Option<PathBuf>,
    #[arg(short = 'o', long)]
    pub output: Option<PathBuf>,
    #[arg(long, default_value_t = 300)]
    pub max_frames: usize,
    #[arg(long)]
    pub no_preview: bool,
    #[arg(long)]
    pub wait_ms: Option<i32>,
}

/// OpenCV 보일러플레이트: open → read → detect → draw → q.
pub fn run_detect(name: &str, args: &DetectArgs, detector: &mut dyn BallDetector) -> Result<()> {
    if let Some(dir) = &args.output {
        fs::create_dir_all(dir).ok();
    }

    let mut source: Box<dyn FrameSource> = if let Some(images) = &args.images {
        Box::new(
            ImageDirSource::open(CameraId(0), images)
                .map_err(anyhow::Error::msg)
                .context("images")?,
        )
    } else if let Some(device) = args.device {
        Box::new(
            OpenCvCapture::from_device(CameraId(0), device)
                .map_err(anyhow::Error::msg)
                .context("device")?,
        )
    } else if let Some(path) = &args.path {
        Box::new(
            OpenCvCapture::from_path(CameraId(0), path)
                .map_err(anyhow::Error::msg)
                .context("path")?,
        )
    } else {
        anyhow::bail!("--images DIR | --path FILE | --device N 중 하나 필요");
    };

    let window = format!("detect:{name}");
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
        let mut panel = frame
            .image
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;

        if let Some(p) = pixel {
            hits += 1;
            println!("{name}: frame={n} pixel=({:.1}, {:.1})", p.x, p.y);
            draw_circle_px(&mut panel, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            if let Some(prev) = prev_pixel {
                draw_circle_px(&mut panel, prev, 6, Scalar::new(0.0, 200.0, 255.0, 0.0), 1)?;
            }
            prev_pixel = last_pixel;
            last_pixel = Some(p);
        } else {
            println!("{name}: frame={n} miss");
        }

        let hit_rate = if n == 0 {
            0.0
        } else {
            100.0 * hits as f64 / (n + 1) as f64
        };
        let lines = [
            format!("{name}  frame={n}"),
            match pixel {
                Some(p) => format!("hit  px=({:.1}, {:.1})", p.x, p.y),
                None => "miss".to_string(),
            },
            format!("hits={hits}/{}  ({hit_rate:.0}%)", n + 1),
            "q/ESC quit".to_string(),
        ];
        draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        draw_cam_label(&mut panel, "cam0", Scalar::new(255.0, 255.0, 255.0, 0.0))?;

        if let Some(dir) = &args.output {
            let out = dir.join(format!("{name}_{n:04}.png"));
            imgcodecs::imwrite(
                out.to_str().context("out path")?,
                &panel,
                &opencv::core::Vector::new(),
            )?;
        }

        if preview {
            match show_bgr(&window, &panel, wait_ms)? {
                PreviewAction::Quit => break,
                PreviewAction::Continue | PreviewAction::Key(_) => {}
            }
        }

        n += 1;
        if args.images.is_none() && n >= args.max_frames {
            break;
        }
    }

    if preview {
        destroy_window(&window);
    }
    println!("{name}: done frames={n} hits={hits}");
    return Ok(());
}
