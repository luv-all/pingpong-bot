//! OpenCV left/right — 격자 · 검출 · 삼각 · 재투영 · stdin→sim.

use std::io::Write;
use std::process::{Child, ChildStdin, Command, Stdio};

use anyhow::{Context, Result, bail};
use opencv::core::{Mat, Point, Scalar};
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::defaults::calibration_path;
use pingpong_bot::defaults::detector_for;
use pingpong_bot::{
    Calibration, CameraId, Detector, Frame, FrameSource, PixelPoint, Point3, Preview,
    PreviewAction, Triangulate, WorldGridParams,
};

use crate::args::Args;

fn open_sources(args: &Args) -> Result<Vec<Box<dyn FrameSource>>> {
    let cam = args.cam.as_cam_cli();
    let (sources, _) = cam
        .open_stereo_input(&args.offline, None)
        .map_err(anyhow::Error::msg)?;
    return Ok(sources);
}

fn reproj_rmse(
    world: Point3,
    hits: &[(CameraId, PixelPoint)],
    calibration: &Calibration,
) -> Option<f64> {
    let errs: Vec<f64> = hits
        .iter()
        .filter_map(|&(id, pix)| {
            let ideal = calibration.params(id)?.project_world(world)?;
            let du = pix.x - ideal.x;
            let dv = pix.y - ideal.y;
            Some(du * du + dv * dv)
        })
        .collect();
    if errs.is_empty() {
        return None;
    }
    return Some((errs.iter().sum::<f64>() / errs.len() as f64).sqrt());
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
    let (max_w, max_h) = Preview::display_fit_bounds().unwrap_or((1920, 1080));
    let fitted = Preview::fit_bgr_downscale(panel, max_w / 2, max_h)?;
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
    let cal_path = calibration_path();
    let calibration = Calibration::load_json(&cal_path).map_err(anyhow::Error::msg)?;
    if calibration.camera_count() < 2 {
        bail!("카메라 ≥2 필요 (got {})", calibration.camera_count());
    }
    let mut sources = open_sources(args)?;
    if sources.len() < 2 {
        bail!("카메라 소스 ≥2 필요 (got {})", sources.len());
    }

    let ids: Vec<CameraId> = sources.iter().map(|s| s.camera_id()).collect();
    let mut detectors: Vec<Detector> = ids
        .iter()
        .map(|&id| detector_for(id))
        .collect::<Result<Vec<_>>>()?;

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
        cal_path.display(),
        ids.iter().map(|c| c.0).collect::<Vec<_>>(),
        args.sim
    );
    println!("g grid  d detect  Space freeze  +/- [] ., grid  q quit");

    loop {
        let frames: Vec<Frame> = if frozen {
            freeze_frames
                .as_ref()
                .expect("freeze")
                .iter()
                .map(|f| {
                    Ok(Frame::new(
                        f.camera_id,
                        f.image
                            .try_clone()
                            .map_err(|e| anyhow::anyhow!("clone: {e}"))?,
                        f.timestamp,
                    ))
                })
                .collect::<Result<Vec<_>>>()?
        } else {
            let Some(imgs) = sources
                .iter_mut()
                .map(|source| source.next_frame())
                .collect::<Option<Vec<_>>>()
            else {
                println!("스트림 종료");
                break;
            };
            freeze_frames = Some(
                imgs.iter()
                    .map(|f| {
                        Ok(Frame::new(
                            f.camera_id,
                            f.image
                                .try_clone()
                                .map_err(|e| anyhow::anyhow!("clone: {e}"))?,
                            f.timestamp,
                        ))
                    })
                    .collect::<Result<Vec<_>>>()?,
            );
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
                Preview::draw_world_grid(&mut panel, params, &grid)?;
            }

            if show_detect {
                if let Some(p) = detectors[i].detect(frame) {
                    hits.push((id, p));
                    Preview::draw_circle_px(
                        &mut panel,
                        p,
                        8,
                        Scalar::new(0.0, 255.0, 0.0, 0.0),
                        2,
                    )?;
                }
            }

            Preview::draw_cam_label(
                &mut panel,
                &format!("cam{}", id.0),
                Scalar::new(255.0, 255.0, 255.0, 0.0),
            )?;
            panels.push((panel, id));
        }

        let world = Triangulate::pixels(&hits, &calibration);
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
                    show_grid, show_detect, frozen, grid.xy_step, grid.z_step, grid.z_layers
                ),
            ];
            Preview::draw_debug_lines(panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            Preview::draw_help_lines(
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
                    Preview::apply_grid_key(&mut grid, k);
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
    Preview::destroy_window(win_left);
    Preview::destroy_window(win_right);
    return Ok(());
}
