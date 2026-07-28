//! OpenCV left/right — 격자 · 검출 · 삼각 · 재투영 · stdin→sim.

use std::io::Write;
use std::path::Path;
use std::process::{Child, ChildStdin, Command, Stdio};

use anyhow::{Context, Result, bail};
use opencv::core::{Mat, Point, Scalar};
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::{
    BallDetector, Calibration, CameraId, Frame, FrameSource, OpenCvCapture, PixelPoint, Point3,
    PreviewAction, WorldGridParams, apply_grid_key, destroy_window, detector_for,
    display_fit_bounds, draw_cam_label, draw_circle_px, draw_debug_lines, draw_help_lines,
    draw_world_grid, fit_bgr_downscale, triangulate_views,
};

use crate::args::Args;

pub fn load_calibration(path: &Path) -> Result<Calibration> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("calibration 읽기: {}", path.display()))?;
    let cal: Calibration = serde_json::from_str(&text)
        .with_context(|| format!("calibration JSON: {}", path.display()))?;
    if cal.camera_count() < 2 {
        bail!("카메라 ≥2 필요 (got {})", cal.camera_count());
    }
    return Ok(cal);
}

fn open_sources(args: &Args) -> Result<Vec<Box<dyn FrameSource>>> {
    let cam = args.cam.as_cam_cli();
    let mut sources = Vec::new();
    if !args.videos.is_empty() {
        let roles = cam.resolve().map_err(anyhow::Error::msg)?;
        for (i, path) in args.videos.iter().enumerate() {
            let id = roles
                .get(i)
                .map(|r| r.camera_id)
                .unwrap_or(CameraId(i as u8));
            let cap = OpenCvCapture::from_path(id, path)
                .map_err(anyhow::Error::msg)
                .with_context(|| format!("video {}", path.display()))?;
            sources.push(Box::new(cap) as Box<dyn FrameSource>);
        }
        return Ok(sources);
    }
    for (_r, src) in cam.open_sources().map_err(anyhow::Error::msg)? {
        sources.push(src);
    }
    return Ok(sources);
}

fn triangulate_pixels(
    hits: &[(CameraId, PixelPoint)],
    calibration: &Calibration,
) -> Option<Point3> {
    if hits.len() < calibration.min_cameras_for_triangulation() {
        return None;
    }
    let mut views = Vec::with_capacity(hits.len());
    for &(id, pix) in hits {
        let params = calibration.params(id)?;
        views.push((params.projection_matrix(), pix));
    }
    return triangulate_views(&views);
}

fn reproj_rmse(
    world: Point3,
    hits: &[(CameraId, PixelPoint)],
    calibration: &Calibration,
) -> Option<f64> {
    if hits.is_empty() {
        return None;
    }
    let mut sum = 0.0;
    let mut n = 0usize;
    for &(id, pix) in hits {
        let params = calibration.params(id)?;
        let ideal = params.project_world(world)?;
        let du = pix.x - ideal.x;
        let dv = pix.y - ideal.y;
        sum += du * du + dv * dv;
        n += 1;
    }
    if n == 0 {
        return None;
    }
    return Some((sum / n as f64).sqrt());
}

fn spawn_sim_child() -> Result<(Child, ChildStdin)> {
    let exe = std::env::current_exe().context("current_exe")?;
    let mut child = Command::new(exe)
        .arg("--sim-child")
        .stdin(Stdio::piped())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .spawn()
        .context("spawn sim child")?;
    let stdin = child.stdin.take().context("sim child stdin")?;
    return Ok((child, stdin));
}

fn send_ball(stdin: &mut ChildStdin, pos: Option<Point3>) {
    let line = match pos {
        Some(p) => format!(r#"{{"x":{},"y":{},"z":{}}}"#, p.x, p.y, p.z),
        None => "hide".to_string(),
    };
    let _ = writeln!(stdin, "{line}");
    let _ = stdin.flush();
}

fn show_panel(window: &str, panel: &Mat, wait_ms: i32) -> Result<PreviewAction> {
    let (max_w, max_h) = display_fit_bounds().unwrap_or((1920, 1080));
    let fitted = fit_bgr_downscale(panel, max_w / 2, max_h)?;
    highgui::imshow(window, &fitted.image)?;
    let key = highgui::wait_key(wait_ms)?;
    if key < 0 {
        return Ok(PreviewAction::Continue);
    }
    let k = key & 0xff;
    if k == i32::from(b'q') || k == 27 {
        return Ok(PreviewAction::Quit);
    }
    return Ok(PreviewAction::Key(key));
}

pub fn run_opencv(args: &Args) -> Result<()> {
    let calibration = load_calibration(&args.calibration)?;
    let mut sources = open_sources(args)?;
    if sources.len() < 2 {
        bail!("카메라 소스 ≥2 필요 (got {})", sources.len());
    }

    let ids: Vec<CameraId> = sources.iter().map(|s| s.camera_id()).collect();
    let mut detectors: Vec<Box<dyn BallDetector>> = Vec::with_capacity(ids.len());
    for &id in &ids {
        detectors.push(Box::new(detector_for(id)?));
    }

    let win_left = "verify:left";
    let win_right = "verify:right";
    highgui::named_window(win_left, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(win_right, highgui::WINDOW_AUTOSIZE)?;

    let mut sim_child: Option<Child> = None;
    let mut sim_stdin: Option<ChildStdin> = None;
    if args.sim {
        match spawn_sim_child() {
            Ok((child, stdin)) => {
                sim_child = Some(child);
                sim_stdin = Some(stdin);
                println!("sim child spawned (stdin XYZ)");
            }
            Err(e) => eprintln!("sim child spawn failed (OpenCV only): {e}"),
        }
    }

    let mut grid = WorldGridParams::default();
    let mut show_grid = true;
    let mut show_detect = true;
    let mut frozen = false;
    let mut freeze_frames: Option<Vec<Frame>> = None;

    println!(
        "verify-stereo — cal={} cams={:?} sim={}",
        args.calibration.display(),
        ids.iter().map(|c| c.0).collect::<Vec<_>>(),
        args.sim
    );
    println!("g grid  d detect  Space freeze  +/- [] ., grid  q quit");

    loop {
        let frames: Vec<Frame> = if frozen {
            let frozen_frames = freeze_frames.as_ref().expect("freeze");
            let mut out = Vec::with_capacity(frozen_frames.len());
            for f in frozen_frames {
                out.push(Frame::new(
                    f.camera_id,
                    f.image
                        .try_clone()
                        .map_err(|e| anyhow::anyhow!("clone: {e}"))?,
                    f.timestamp,
                ));
            }
            out
        } else {
            let mut imgs = Vec::with_capacity(sources.len());
            let mut ok = true;
            for source in sources.iter_mut() {
                let Some(frame) = source.next_frame() else {
                    ok = false;
                    break;
                };
                imgs.push(frame);
            }
            if !ok {
                println!("스트림 종료");
                break;
            }
            let mut stored = Vec::with_capacity(imgs.len());
            for f in &imgs {
                stored.push(Frame::new(
                    f.camera_id,
                    f.image
                        .try_clone()
                        .map_err(|e| anyhow::anyhow!("clone: {e}"))?,
                    f.timestamp,
                ));
            }
            freeze_frames = Some(stored);
            imgs
        };

        let mut hits: Vec<(CameraId, PixelPoint)> = Vec::new();
        let mut panels: Vec<(Mat, CameraId)> = Vec::with_capacity(frames.len());

        for (i, frame) in frames.iter().enumerate() {
            let id = frame.camera_id;
            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            let Some(params) = calibration.params(id) else {
                bail!("calibration에 cam{} 없음", id.0);
            };

            if show_grid {
                draw_world_grid(&mut panel, params, grid)?;
            }

            if show_detect {
                if let Some(p) = detectors[i].detect(frame) {
                    hits.push((id, p));
                    draw_circle_px(&mut panel, p, 8, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
                }
            }

            draw_cam_label(
                &mut panel,
                &format!("cam{}", id.0),
                Scalar::new(255.0, 255.0, 255.0, 0.0),
            )?;
            panels.push((panel, id));
        }

        let world = triangulate_pixels(&hits, &calibration);
        let rmse = world.and_then(|w| reproj_rmse(w, &hits, &calibration));

        if let Some(stdin) = &mut sim_stdin {
            send_ball(stdin, world);
        }

        // Prefer index over `panels.iter_mut()`: Mat::iter_mut confuses rust-analyzer on Vec<(Mat, _)>.
        for i in 0..panels.len() {
            let id = panels[i].1;
            let panel = &mut panels[i].0;
            let Some(params) = calibration.params(id) else {
                continue;
            };
            if let Some(w) = world {
                if let Some(ideal) = params.project_world(w) {
                    let p = Point::new(ideal.x.round() as i32, ideal.y.round() as i32);
                    imgproc::draw_marker(
                        panel,
                        p,
                        Scalar::new(255.0, 0.0, 255.0, 0.0),
                        imgproc::MARKER_CROSS,
                        16,
                        2,
                        imgproc::LINE_AA,
                    )?;
                    if let Some((_, hit)) = hits.iter().find(|(hid, _)| *hid == id) {
                        let c = Point::new(hit.x.round() as i32, hit.y.round() as i32);
                        imgproc::line(
                            panel,
                            c,
                            p,
                            Scalar::new(0.0, 255.0, 255.0, 0.0),
                            1,
                            imgproc::LINE_AA,
                            0,
                        )?;
                    }
                }
            }

            let xyz = match world {
                Some(w) => format!("xyz=({:.3},{:.3},{:.3})", w.x, w.y, w.z),
                None => "xyz=—".into(),
            };
            let rmse_s = match rmse {
                Some(r) => format!("reproj={r:.1}px"),
                None => "reproj=—".into(),
            };
            let lines = [
                format!("VERIFY hits={}/2 {xyz} {rmse_s}", hits.len()),
                format!(
                    "grid={} detect={} freeze={}  xy={:.2} z={:.2} L{}",
                    show_grid,
                    show_detect,
                    frozen,
                    grid.xy_step,
                    grid.z_step,
                    grid.z_layers
                ),
            ];
            draw_debug_lines(panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            draw_help_lines(
                panel,
                &["g grid  d detect  Space freeze", "+/- [] ., grid  q quit"],
                Scalar::new(0.0, 255.0, 80.0, 0.0),
            )?;
        }

        let wait = if frozen { 30 } else { 1 };
        let _ = show_panel(win_left, &panels[0].0, 1)?;
        let action = show_panel(win_right, &panels[1].0, wait)?;

        match action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(key) => {
                let k = key & 0xff;
                if k == i32::from(b'g') || k == i32::from(b'G') {
                    show_grid = !show_grid;
                } else if k == i32::from(b'd') || k == i32::from(b'D') {
                    show_detect = !show_detect;
                } else if k == i32::from(b' ') {
                    frozen = !frozen;
                    if !frozen {
                        freeze_frames = None;
                    }
                } else {
                    apply_grid_key(&mut grid, k);
                }
            }
        }
    }

    if let Some(mut stdin) = sim_stdin.take() {
        send_ball(&mut stdin, None);
        drop(stdin); // EOF → 자식 stdin 루프 종료
    }
    if let Some(mut child) = sim_child.take() {
        let _ = child.kill();
        let _ = child.wait();
    }
    destroy_window(win_left);
    destroy_window(win_right);
    return Ok(());
}
