//! 같은 입력에 detect 4종을 한 번에 돌려 비교한다.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use opencv::core::{Point, Scalar, Vector};
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::{
    BallDetector, BgSubDetector, CameraId, ColormaskConfig, ColormaskDetector, ContourDetector,
    FrameSource, ImageDirSource, OpenCvCapture, PixelPoint, PreviewAction, RoiDetector,
    destroy_window, draw_debug_lines, show_bgr,
};

#[derive(Parser, Debug)]
#[command(
    name = "detect_compare",
    about = "같은 프레임에 colormask/bgsub/contour/roi를 한 번에 비교"
)]
struct Args {
    /// 이미지 폴더
    #[arg(long)]
    images: Option<PathBuf>,
    /// 웹캠 인덱스
    #[arg(long)]
    device: Option<i32>,
    /// 동영상 파일
    #[arg(long)]
    path: Option<PathBuf>,
    /// 오버레이 저장 디렉터리 (frame_NNNN.png = 2x2 그리드)
    #[arg(short = 'o', long)]
    output: Option<PathBuf>,
    #[arg(long, default_value_t = 300)]
    max_frames: usize,
    #[arg(long)]
    no_preview: bool,
    #[arg(long)]
    wait_ms: Option<i32>,
}

fn open_source(args: &Args) -> Result<Box<dyn FrameSource>> {
    if let Some(images) = &args.images {
        return Ok(Box::new(
            ImageDirSource::open(CameraId(0), images)
                .map_err(anyhow::Error::msg)
                .context("images")?,
        ));
    }
    if let Some(device) = args.device {
        return Ok(Box::new(
            OpenCvCapture::from_device(CameraId(0), device)
                .map_err(anyhow::Error::msg)
                .context("device")?,
        ));
    }
    if let Some(path) = &args.path {
        return Ok(Box::new(
            OpenCvCapture::from_path(CameraId(0), path)
                .map_err(anyhow::Error::msg)
                .context("path")?,
        ));
    }
    anyhow::bail!("--images DIR | --path FILE | --device N 중 하나 필요");
}

fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(dir) = &args.output {
        fs::create_dir_all(dir)?;
    }

    let mut source = open_source(&args)?;
    let mut detectors: Vec<(&str, Box<dyn BallDetector>, Scalar)> = vec![
        (
            "colormask",
            Box::new(ColormaskDetector::new(ColormaskConfig::default())),
            Scalar::new(0.0, 255.0, 0.0, 0.0),
        ),
        (
            "bgsub",
            Box::new(BgSubDetector::new()),
            Scalar::new(0.0, 255.0, 255.0, 0.0),
        ),
        (
            "contour",
            Box::new(ContourDetector::new()),
            Scalar::new(255.0, 128.0, 0.0, 0.0),
        ),
        (
            "roi",
            Box::new(RoiDetector::new()),
            Scalar::new(0.0, 0.0, 255.0, 0.0),
        ),
    ];

    let mut hits = [0usize; 4];
    let mut n = 0usize;
    let window = "detect:compare";
    let wait_ms = args
        .wait_ms
        .unwrap_or(if args.path.is_some() || args.images.is_some() {
            33
        } else {
            1
        });

    println!("frame\tcolormask\tbgsub\tcontour\troi");
    while let Some(frame) = source.next_frame() {
        let mut cells = Vec::with_capacity(4);
        let mut row = format!("{n}");
        let mut summary = Vec::new();

        for (i, (name, detector, color)) in detectors.iter_mut().enumerate() {
            let pixel = detector.detect(&frame);
            match pixel {
                Some(p) => {
                    hits[i] += 1;
                    row.push_str(&format!("\t{:.0},{:.0}", p.x, p.y));
                    summary.push(format!("{name}=({:.0},{:.0})", p.x, p.y));
                }
                None => {
                    row.push_str("\t-");
                    summary.push(format!("{name}=miss"));
                }
            }

            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            imgproc::put_text(
                &mut panel,
                name,
                Point::new(8, 24),
                imgproc::FONT_HERSHEY_SIMPLEX,
                0.7,
                *color,
                2,
                imgproc::LINE_8,
                false,
            )?;
            if let Some(p) = pixel {
                draw_hit(&mut panel, p, *color)?;
            }
            cells.push(panel);
        }
        println!("{row}");

        let grid = make_grid(&cells)?;
        let mut display = grid.try_clone()?;
        let mut lines = vec![format!("compare frame={n}")];
        lines.extend(summary);
        lines.push(format!(
            "hits {}/{}/{}/{}  q/ESC quit",
            hits[0], hits[1], hits[2], hits[3]
        ));
        draw_debug_lines(&mut display, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;

        if let Some(dir) = &args.output {
            let out = dir.join(format!("compare_{n:04}.png"));
            imgcodecs::imwrite(out.to_str().context("out")?, &display, &Vector::new())?;
        }

        if !args.no_preview {
            match show_bgr(window, &display, wait_ms)? {
                PreviewAction::Quit => break,
                PreviewAction::Continue | PreviewAction::Key(_) => {}
            }
        }

        n += 1;
        if args.images.is_none() && n >= args.max_frames {
            break;
        }
    }

    if !args.no_preview {
        destroy_window(window);
    }

    println!(
        "# done frames={n} hits colormask={} bgsub={} contour={} roi={}",
        hits[0], hits[1], hits[2], hits[3]
    );
    return Ok(());
}

fn draw_hit(img: &mut Mat, pixel: PixelPoint, color: Scalar) -> Result<()> {
    imgproc::circle(
        img,
        Point::new(pixel.x as i32, pixel.y as i32),
        10,
        color,
        2,
        imgproc::LINE_8,
        0,
    )?;
    return Ok(());
}

fn make_grid(cells: &[Mat]) -> Result<Mat> {
    ensure_same_size(cells)?;
    let mut top = Mat::default();
    let mut bottom = Mat::default();
    let mut out = Mat::default();
    opencv::core::hconcat(
        &Vector::<Mat>::from_iter([cells[0].clone(), cells[1].clone()]),
        &mut top,
    )?;
    opencv::core::hconcat(
        &Vector::<Mat>::from_iter([cells[2].clone(), cells[3].clone()]),
        &mut bottom,
    )?;
    opencv::core::vconcat(&Vector::<Mat>::from_iter([top, bottom]), &mut out)?;
    return Ok(out);
}

fn ensure_same_size(cells: &[Mat]) -> Result<()> {
    let s0 = cells[0].size()?;
    for (i, c) in cells.iter().enumerate().skip(1) {
        let s = c.size()?;
        if s != s0 {
            anyhow::bail!("panel size mismatch: 0={s0:?} {i}={s:?}");
        }
    }
    return Ok(());
}
