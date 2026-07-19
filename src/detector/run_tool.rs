//! detect_* 툴 공용 실행 (캡처 창 + 디버그 오버레이).

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};
use opencv::core::Scalar;
use opencv::imgcodecs;
use opencv::prelude::*;

use crate::camera::preview::{
    PreviewAction, destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines, show_bgr,
};
use crate::{BallDetector, CameraId, FrameSource, ImageDirSource, OpenCvCapture, PixelPoint};

/// detect_* CLI와 동일한 입력.
#[derive(Debug, Clone)]
pub struct DetectToolOptions {
    pub images: Option<PathBuf>,
    pub device: Option<i32>,
    pub path: Option<PathBuf>,
    pub output: Option<PathBuf>,
    pub max_frames: usize,
    /// false 면 highgui 생략
    pub preview: bool,
    /// 파일 재생 시 waitKey ms (0이면 FPS 추정)
    pub wait_ms: Option<i32>,
}

pub fn open_frame_source(
    images: &Option<PathBuf>,
    device: Option<i32>,
    path: &Option<PathBuf>,
) -> Result<Box<dyn FrameSource>> {
    if let Some(dir) = images {
        return Ok(Box::new(
            ImageDirSource::open(CameraId(0), dir)
                .map_err(anyhow::Error::msg)
                .context("images")?,
        ));
    }
    if let Some(dev) = device {
        return Ok(Box::new(
            OpenCvCapture::from_device(CameraId(0), dev)
                .map_err(anyhow::Error::msg)
                .context("device")?,
        ));
    }
    if let Some(p) = path {
        return Ok(Box::new(
            OpenCvCapture::from_path(CameraId(0), p)
                .map_err(anyhow::Error::msg)
                .context("path")?,
        ));
    }
    anyhow::bail!("--images DIR | --path FILE | --device N 중 하나 필요");
}

/// 검출 실험 루프: 콘솔 + (옵션) 프리뷰 + `-o` PNG.
pub fn run_detect_tool(
    name: &str,
    opts: &DetectToolOptions,
    detector: &mut dyn BallDetector,
) -> Result<()> {
    if let Some(dir) = &opts.output {
        fs::create_dir_all(dir).ok();
    }

    let mut source = open_frame_source(&opts.images, opts.device, &opts.path)?;
    let window = format!("detect:{name}");
    let wait_ms = opts
        .wait_ms
        .unwrap_or(if opts.path.is_some() || opts.images.is_some() {
            33
        } else {
            1
        });

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

        if let Some(dir) = &opts.output {
            let out = dir.join(format!("{name}_{n:04}.png"));
            imgcodecs::imwrite(
                out.to_str().context("out path")?,
                &panel,
                &opencv::core::Vector::new(),
            )?;
        }

        if opts.preview {
            match show_bgr(&window, &panel, wait_ms)? {
                PreviewAction::Quit => break,
                PreviewAction::Continue => {}
            }
        }

        n += 1;
        if opts.images.is_none() && n >= opts.max_frames {
            break;
        }
    }

    if opts.preview {
        destroy_window(&window);
    }
    println!("{name}: done frames={n} hits={hits}");
    return Ok(());
}
