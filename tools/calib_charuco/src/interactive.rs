//! 라이브/영상 인터랙티브 보정: Space → 코너 확인 → s 저장 → q 시 calibrate.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use opencv::core::{Mat, Scalar};
use opencv::imgcodecs;
use opencv::prelude::*;
use pingpong_bot::{
    CameraId, FrameSource, MIN_CHARUCO_CORNERS, OpenCvCapture, PreviewAction, destroy_window,
    detect_and_draw_charuco, draw_debug_lines, draw_help_lines, show_bgr,
};

use crate::args::{Args, board_spec, resolve_output};
use crate::cli;

fn default_images_dir(camera_id: u8) -> PathBuf {
    return PathBuf::from(format!("calib_frames/cam{camera_id}"));
}

fn count_images(dir: &PathBuf) -> usize {
    let Ok(rd) = fs::read_dir(dir) else {
        return 0;
    };
    return rd
        .filter_map(|e| e.ok())
        .filter(|e| {
            matches!(
                e.path().extension().and_then(|x| x.to_str()),
                Some("png" | "jpg" | "jpeg")
            )
        })
        .count();
}

fn next_save_path(dir: &PathBuf, index: usize) -> PathBuf {
    return dir.join(format!("{index:04}.png"));
}

struct ReviewFrame {
    raw: Mat,
    overlay: Mat,
    ok: bool,
    corners: usize,
    markers: usize,
}

pub fn run(args: &Args) -> Result<()> {
    if args.device.is_some() && args.path.is_some() {
        bail!("--device 와 --path 를 같이 쓰지 마세요");
    }

    let images_dir = args
        .images_dir
        .clone()
        .unwrap_or_else(|| default_images_dir(args.camera_id));
    fs::create_dir_all(&images_dir).with_context(|| format!("mkdir {}", images_dir.display()))?;

    let cam_id = CameraId(args.camera_id);
    let mut source: Box<dyn FrameSource> = if let Some(path) = &args.path {
        Box::new(
            OpenCvCapture::from_path(cam_id, path)
                .map_err(anyhow::Error::msg)
                .context("path")?,
        )
    } else {
        let device = args.device.unwrap_or(0);
        Box::new(
            OpenCvCapture::from_device(cam_id, device)
                .map_err(anyhow::Error::msg)
                .with_context(|| format!("device {device}"))?,
        )
    };

    let window = "calib:charuco";
    let spec = board_spec(args);
    let output = resolve_output(args);
    let mut review: Option<ReviewFrame> = None;
    let mut save_index = count_images(&images_dir);

    println!(
        "인터랙티브 보정 — dir={}  min_frames={}  -o {}",
        images_dir.display(),
        args.min_frames,
        output.display()
    );
    println!("Space=스냅+코너  s=저장  n=생략  q=종료(+calib)");

    loop {
        let action = if let Some(ref rev) = review {
            let mut panel = rev
                .overlay
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            let status = if rev.ok {
                format!(
                    "REVIEW OK  corners={} markers={} (>= {MIN_CHARUCO_CORNERS})",
                    rev.corners, rev.markers
                )
            } else {
                format!(
                    "REVIEW FAIL  corners={} markers={} - skip with n",
                    rev.corners, rev.markers
                )
            };
            let color = if rev.ok {
                Scalar::new(0.0, 255.0, 0.0, 0.0)
            } else {
                Scalar::new(0.0, 0.0, 255.0, 0.0)
            };
            let lines = [status, format!("saved={save_index}")];
            draw_debug_lines(&mut panel, &lines, color)?;
            draw_help_lines(
                &mut panel,
                &["s save", "n skip", "q quit"],
                Scalar::new(0.0, 255.0, 80.0, 0.0),
            )?;
            show_bgr(window, &panel, 30)?
        } else {
            let Some(frame) = source.next_frame() else {
                println!("입력 스트림 종료");
                break;
            };
            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            let lines = [format!(
                "LIVE  saved={save_index} (need >= {})",
                args.min_frames
            )];
            draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            draw_help_lines(
                &mut panel,
                &["Space snap+detect", "q quit(+calib)"],
                Scalar::new(0.0, 255.0, 80.0, 0.0),
            )?;
            // space를 받기 위해 라이브 프레임도 잠시 들고 있음
            let action = show_bgr(window, &panel, 1)?;
            if matches!(action, PreviewAction::Key(k) if k == i32::from(b' ')) {
                let raw = frame
                    .image
                    .try_clone()
                    .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
                let (overlay, det) =
                    detect_and_draw_charuco(&raw, spec).map_err(anyhow::Error::msg)?;
                println!(
                    "snap: corners={} markers={} ok={}",
                    det.corners, det.markers, det.ok
                );
                review = Some(ReviewFrame {
                    raw,
                    overlay,
                    ok: det.ok,
                    corners: det.corners,
                    markers: det.markers,
                });
                continue;
            }
            action
        };

        match action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(k) if k == i32::from(b' ') => {
                // review 중 Space는 무시 (라이브에서만)
            }
            PreviewAction::Key(k) if k == i32::from(b's') || k == i32::from(b'S') => {
                let Some(rev) = review.take() else {
                    continue;
                };
                if !rev.ok {
                    println!("저장 거부 — 코너 부족 (n으로 생략)");
                    continue;
                }
                let out = next_save_path(&images_dir, save_index);
                let path_str = out.to_str().context("non-utf8 save path")?;
                imgcodecs::imwrite(path_str, &rev.raw, &opencv::core::Vector::new())
                    .with_context(|| format!("imwrite {}", out.display()))?;
                println!("saved {} (corners={})", out.display(), rev.corners);
                save_index += 1;
            }
            PreviewAction::Key(k) if k == i32::from(b'n') || k == i32::from(b'N') => {
                if review.take().is_some() {
                    println!("skip");
                }
            }
            PreviewAction::Key(_) => {}
        }
    }

    destroy_window(window);

    let n = count_images(&images_dir);
    if n < args.min_frames {
        println!(
            "저장 {n}장 < min_frames={} — calibrate 생략. 이후:\n  \
             cargo run -p calib-charuco -- --from-images {} -o {}",
            args.min_frames,
            images_dir.display(),
            output.display()
        );
        return Ok(());
    }

    println!("calibrate from {n} images in {} …", images_dir.display());
    return cli::from_images(&images_dir, args);
}
