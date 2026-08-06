//! OpenCV left/right — 격자 · 검출 · 삼각 · EKF · 재투영 · stdin→sim.
//!
//! 삼각측량 생값과 EKF 출력을 나란히 그려 게이트가 무엇을 걸렀는지 본다.
//! **마젠타 ×** 생 재투영(통과) · **빨강 ×** 게이트가 막은 생값 ·
//! **시안 ○** EKF 재투영. sim 창은 주황 공 = EKF, 반투명 공 = 생값.

use std::io::Write;
use std::process::{Child, ChildStdin, Command, Stdio};

use anyhow::{Context, Result, bail};
use opencv::core::{Mat, Point, Scalar};
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::Point3;
use pingpong_bot::camera;
use pingpong_bot::camera::{
    Calibration, Frame, FrameSource, Preview, PreviewAction, WorldGridParams,
};
use pingpong_bot::defaults::calibration_path;
use pingpong_bot::defaults::detector_for;
use pingpong_bot::defaults::vision::fit as fit_params;
use pingpong_bot::vision::Detector;
use pingpong_bot::vision::{Fit, Outcome, triggers};

use crate::args::Args;
use crate::msg::BallMsg;

/// 게이트를 통과한 생 삼각측량 재투영.
const COLOR_RAW: Scalar = Scalar::new(255.0, 0.0, 255.0, 0.0);
/// 게이트가 막은 생 삼각측량 재투영.
const COLOR_REJECT: Scalar = Scalar::new(0.0, 0.0, 255.0, 0.0);
/// EKF 출력 재투영.
const COLOR_EKF: Scalar = Scalar::new(255.0, 255.0, 0.0, 0.0);

fn open_sources(args: &Args) -> Result<Vec<Box<dyn FrameSource>>> {
    let cam = args.cam.as_cam_cli();
    let (sources, _) = cam
        .open_stereo_input(&args.offline, None)
        .map_err(anyhow::Error::msg)?;
    return Ok(sources);
}

fn reproj_rmse(
    world: Point3,
    hits: &[(camera::Id, camera::Pixel)],
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

fn send_ball(stdin: &mut ChildStdin, raw: Option<Point3>, ekf: Option<Point3>) {
    let msg = BallMsg {
        raw: raw.map(Into::into),
        ekf: ekf.map(Into::into),
    };
    let _ = writeln!(stdin, "{}", msg.to_line());
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

    let ids: Vec<camera::Id> = sources.iter().map(|s| s.camera_id()).collect();
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

    // 검출이 튀어도 궤적이 끊기지 않게 하는 게이팅 필터. `e`로 끄면 생값만 본다.
    //
    // 트리거는 이 툴이 안 쓴다 (예측 궤적을 안 그린다). 뭐든 하나는 있어야 해서 네트로 둔다.
    let mut ekf = Fit::new(
        &calibration,
        Box::new(triggers::PlaneCrossing {
            y: pingpong_bot::constants::table::LENGTH_Y * 0.5,
        }),
    );
    let mut epoch: Option<std::time::Instant> = None;
    let mut ekf_enabled = true;
    let mut last_px: Option<f64> = None;
    let mut rejected_this_frame = false;
    let mut rejected_total: u64 = 0;
    let mut seeds_total: u64 = 0;

    println!(
        "verify-stereo — cal={} cams={:?} sim={}",
        cal_path.display(),
        ids.iter().map(|c| c.0).collect::<Vec<_>>(),
        args.sim
    );
    println!("g grid  d detect  e ekf  Space freeze  +/- [] ., grid  q quit");

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

        let mut hits: Vec<(camera::Id, camera::Pixel)> = Vec::new();
        let mut panels: Vec<(Mat, camera::Id)> = Vec::with_capacity(frames.len());

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
                let found = detectors[i].detect(frame, None)?;
                // 동결 중에는 같은 프레임이 반복되므로 필터를 돌리지 않는다.
                if ekf_enabled && !frozen {
                    let base = *epoch.get_or_insert(frame.timestamp);
                    let t = frame.timestamp.saturating_duration_since(base);
                    match ekf.observe(id, found, t) {
                        Outcome::Rejected { px } => {
                            rejected_total += 1;
                            rejected_this_frame = true;
                            last_px = Some(px);
                        }
                        Outcome::Seeded => seeds_total += 1,
                        Outcome::Accepted | Outcome::Idle => {}
                    }
                }
                if let Some(p) = found.map(|candidate| candidate.pixel) {
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

        let world = camera::Triangulate::pixels(&hits, &calibration);
        let rmse = world.and_then(|w| reproj_rmse(w, &hits, &calibration));

        let state = ekf_enabled
            .then(|| ekf.measured().last().copied())
            .flatten();
        let filtered = state.map(|s| s.position);
        let rejected = std::mem::take(&mut rejected_this_frame);

        if let Some(stdin) = &mut sim_stdin {
            send_ball(stdin, world, filtered);
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
                        if rejected { COLOR_REJECT } else { COLOR_RAW },
                        imgproc::MARKER_CROSS,
                        16,
                        2,
                        imgproc::LINE_AA,
                    )?;
                    if rejected {
                        imgproc::put_text(
                            panel,
                            "REJECT",
                            Point::new(p.x + 12, p.y - 12),
                            imgproc::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            COLOR_REJECT,
                            1,
                            imgproc::LINE_AA,
                            false,
                        )?;
                    }
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

            // EKF 출력 — 거부된 프레임에서도 필터는 예측으로 이어진다.
            if let Some(f) = filtered {
                if let Some(ideal) = params.project_world(f) {
                    let p = Point::new(ideal.x.round() as i32, ideal.y.round() as i32);
                    imgproc::circle(panel, p, 10, COLOR_EKF, 2, imgproc::LINE_AA, 0)?;
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
            let ekf_xyz = match filtered {
                Some(f) => format!("ekf=({:.3},{:.3},{:.3})", f.x, f.y, f.z),
                None if ekf_enabled => "ekf=—".into(),
                None => "ekf=off".into(),
            };
            let speed = match state {
                Some(s) => format!("|v|={:.1}m/s", s.velocity.norm()),
                None => "|v|=—".into(),
            };
            let d2 = match last_px {
                Some(d) => format!(
                    "resid={d:.1}/{:.1}px",
                    fit_params::OUTLIER_SIGMA * fit_params::SIGMA_PX
                ),
                None => "d2=—".into(),
            };
            let lines = [
                format!("VERIFY hits={}/2 {xyz} {rmse_s}", hits.len()),
                format!(
                    "{ekf_xyz} {speed} {d2} seq={} seeds={seeds_total} rej={rejected_total}",
                    ekf.seq()
                ),
                format!(
                    "grid={} detect={} freeze={}  xy={:.2} z={:.2} L{}",
                    show_grid, show_detect, frozen, grid.xy_step, grid.z_step, grid.z_layers
                ),
            ];
            Preview::draw_debug_lines(panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            Preview::draw_help_lines(
                panel,
                &[
                    "g grid  d detect  e ekf  Space freeze",
                    "magenta=raw red=rejected cyan=ekf",
                    "+/- [] ., grid  q quit",
                ],
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
                } else if k == i32::from(b'e') || k == i32::from(b'E') {
                    ekf_enabled = !ekf_enabled;
                    ekf.drop_track();
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
        send_ball(&mut stdin, None, None);
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
