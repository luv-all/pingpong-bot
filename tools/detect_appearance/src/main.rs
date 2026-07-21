//! appearance 레이어 비교 — colormask | contour 좌우 패널.
//!
//! fuse/ROI는 [detect-full](../detect_full/README.md).

mod cli;

use anyhow::{Context, Result};
use clap::Parser;
use opencv::core::Scalar;
use opencv::imgcodecs;
use pingpong_bot::{
    CameraId, ColormaskDetector, ContourDetector, FrameSource, ImageDirSource, OpenCvCapture,
    PixelPoint, PreviewAction, destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines,
    draw_help_lines, hstack_bgr, load_vision_from_config, show_bgr,
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
    let vision = load_vision_from_config(&args.config)?;
    let mut colormask = ColormaskDetector::try_from(&vision.appearance.colormask)?;
    let mut contour = ContourDetector::from(&vision.scorer);
    println!(
        "appearance colormask|contour (from {})",
        args.config.display()
    );

    let window = "detect:appearance";
    let wait_ms = args
        .wait_ms
        .unwrap_or(if args.path.is_some() || args.images.is_some() {
            33
        } else {
            1
        });
    let preview = !args.no_preview;

    let mut n = 0usize;
    let mut hits = [0usize; 2];

    while let Some(frame) = source.next_frame() {
        let (cm_px, mut cm_mask) = colormask.detect_debug(&frame);
        let (ct_px, mut ct_mask) = contour.detect_debug(&frame);

        if let Some(p) = cm_px {
            hits[0] += 1;
            draw_circle_px(&mut cm_mask, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
        }
        if let Some(p) = ct_px {
            hits[1] += 1;
            draw_circle_px(&mut ct_mask, p, 10, Scalar::new(255.0, 128.0, 0.0, 0.0), 2)?;
        }

        draw_cam_label(
            &mut cm_mask,
            "colormask",
            Scalar::new(0.0, 255.0, 0.0, 0.0),
        )?;
        draw_cam_label(&mut ct_mask, "contour", Scalar::new(255.0, 128.0, 0.0, 0.0))?;

        let mut mosaic = hstack_bgr(&[cm_mask, ct_mask])?;
        let lines = [
            format!("appearance frame={n}"),
            format!(
                "colormask={}  contour={}",
                fmt_px(cm_px),
                fmt_px(ct_px)
            ),
            format!("hits {}/{}", hits[0], hits[1]),
        ];
        draw_debug_lines(&mut mosaic, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        draw_help_lines(
            &mut mosaic,
            &["q/ESC quit"],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;

        println!(
            "frame={n}\tcolormask={}\tcontour={}",
            fmt_px(cm_px),
            fmt_px(ct_px)
        );

        if let Some(dir) = &args.output {
            let out = dir.join(format!("appearance_{n:04}.png"));
            imgcodecs::imwrite(
                out.to_str().context("out path")?,
                &mosaic,
                &opencv::core::Vector::new(),
            )?;
        }

        if preview {
            match show_bgr(window, &mosaic, wait_ms)? {
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
        destroy_window(window);
    }
    println!(
        "done frames={n} hits colormask={} contour={}",
        hits[0], hits[1]
    );
    return Ok(());
}

fn fmt_px(p: Option<PixelPoint>) -> String {
    return match p {
        Some(p) => format!("({:.0},{:.0})", p.x, p.y),
        None => "-".to_string(),
    };
}
