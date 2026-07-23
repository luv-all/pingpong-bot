//! 라이브: Space 스냅 → 랜드마크 순서 클릭 → s 저장(PnP).

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, bail};
use opencv::core::{Mat, Point, Scalar};
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::{
    CameraId, FrameSource, OpenCvCapture, PixelPoint, PreviewAction, TABLE_LANDMARK_COUNT,
    TableLandmark, calibrate_table_pnp, destroy_window, draw_debug_lines, draw_help_lines, show_bgr,
    table_landmark_mesh_edges, table_landmarks,
};

use crate::args::Args;
use crate::cli;

pub fn run(args: &Args) -> Result<()> {
    if args.device.is_some() && args.path.is_some() {
        bail!("--device 와 --path 를 같이 쓰지 마세요");
    }

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

    let window = "calib:table-pnp";
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

    let marks = table_landmarks();
    let mut frozen = false;
    let mut freeze_img: Option<Mat> = None;
    let mut clicks: Vec<PixelPoint> = Vec::new();

    println!(
        "table-PnP — cam={} fov_y={} max_rmse={}",
        args.camera_id, args.fov_y, args.max_rmse
    );
    println!("Space=freeze  LMB=click  z=undo  c=clear  s=solve+save  n=live  q=quit");
    for (i, m) in marks.iter().enumerate() {
        println!("  {}: {}", i + 1, m.prompt);
    }

    loop {
        if frozen {
            let mut q = pending.lock().expect("pending");
            for (x, y) in q.drain(..) {
                if clicks.len() < TABLE_LANDMARK_COUNT {
                    clicks.push(PixelPoint::new(f64::from(x), f64::from(y)));
                    println!(
                        "click {}/{} → ({x},{y})  {}",
                        clicks.len(),
                        TABLE_LANDMARK_COUNT,
                        marks[clicks.len() - 1].id
                    );
                }
            }
        } else {
            pending.lock().expect("pending").clear();
        }

        let frame_img = if frozen {
            freeze_img
                .as_ref()
                .expect("freeze_img")
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?
        } else {
            let Some(frame) = source.next_frame() else {
                println!("입력 스트림 종료");
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

        let mut panel = frame_img
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
        if frozen {
            draw_clicks(&mut panel, &clicks, &marks)?;
            let next = if clicks.len() < TABLE_LANDMARK_COUNT {
                marks[clicks.len()].prompt.to_string()
            } else {
                format!("all {TABLE_LANDMARK_COUNT} clicked - press s")
            };
            let lines = [
                format!("REVIEW clicks={}/{}", clicks.len(), TABLE_LANDMARK_COUNT),
                next,
            ];
            draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            draw_help_lines(
                &mut panel,
                &["LMB click", "z undo", "c clear", "s solve", "n live", "q quit"],
                Scalar::new(0.0, 255.0, 80.0, 0.0),
            )?;
        } else {
            draw_debug_lines(
                &mut panel,
                &["LIVE - Space to freeze"],
                Scalar::new(0.0, 255.0, 255.0, 0.0),
            )?;
            draw_help_lines(
                &mut panel,
                &["Space freeze", "q quit"],
                Scalar::new(0.0, 255.0, 80.0, 0.0),
            )?;
        }

        let wait = if frozen { 30 } else { 1 };
        let action = show_bgr(window, &panel, wait)?;
        match action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(k) => {
                let key = k & 0xff;
                if !frozen && key == i32::from(b' ') {
                    if freeze_img.is_some() {
                        frozen = true;
                        clicks.clear();
                        println!("frozen — click landmarks in order");
                    }
                } else if key == i32::from(b'n') || key == i32::from(b'N') {
                    frozen = false;
                    clicks.clear();
                } else if key == i32::from(b'z') || key == i32::from(b'Z') {
                    clicks.pop();
                } else if key == i32::from(b'c') || key == i32::from(b'C') {
                    clicks.clear();
                } else if (key == i32::from(b's') || key == i32::from(b'S')) && frozen {
                    if clicks.len() != TABLE_LANDMARK_COUNT {
                        println!(
                            "클릭 {}/{} - 모두 찍으세요",
                            clicks.len(),
                            TABLE_LANDMARK_COUNT
                        );
                        continue;
                    }
                    let img = freeze_img.as_ref().expect("freeze_img");
                    let w = img.cols().max(1) as u32;
                    let h = img.rows().max(1) as u32;
                    let result =
                        calibrate_table_pnp(cam_id, None, w, h, args.fov_y, &clicks)
                            .map_err(anyhow::Error::msg)?;
                    println!(
                        "PnP candidates={} rmse={:.2}px",
                        result.candidates, result.reproj_rmse
                    );
                    if result.reproj_rmse > args.max_rmse {
                        println!(
                            "FAIL rmse {:.2} > {} — 다시 클릭 (z/c) 또는 --fov-y",
                            result.reproj_rmse, args.max_rmse
                        );
                        continue;
                    }
                    cli::write_result(args, result.params, result.reproj_rmse, result.candidates)?;
                    break;
                }
            }
        }
    }

    destroy_window(window);
    return Ok(());
}

fn draw_clicks(panel: &mut Mat, clicks: &[PixelPoint], marks: &[TableLandmark]) -> Result<()> {
    // 탁구대 메시: 양 끝이 모두 찍힌 선분만 (클릭 순서 폴리라인 아님)
    let edge_color = Scalar::new(255.0, 128.0, 0.0, 0.0);
    for &(a_i, b_i) in table_landmark_mesh_edges() {
        if a_i >= clicks.len() || b_i >= clicks.len() {
            continue;
        }
        let a = Point::new(clicks[a_i].x.round() as i32, clicks[a_i].y.round() as i32);
        let b = Point::new(clicks[b_i].x.round() as i32, clicks[b_i].y.round() as i32);
        imgproc::line(panel, a, b, edge_color, 1, imgproc::LINE_AA, 0)?;
    }

    for (i, px) in clicks.iter().enumerate() {
        let p = Point::new(px.x.round() as i32, px.y.round() as i32);
        imgproc::circle(
            panel,
            p,
            6,
            Scalar::new(0.0, 255.0, 0.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
        let label = format!("{}:{}", i + 1, marks[i].id);
        imgproc::put_text(
            panel,
            &label,
            Point::new(p.x + 8, p.y - 8),
            imgproc::FONT_HERSHEY_SIMPLEX,
            0.5,
            Scalar::new(0.0, 255.0, 0.0, 0.0),
            1,
            imgproc::LINE_AA,
            false,
        )?;
    }
    return Ok(());
}
