//! 라이브: Space 스냅 → 8점 클릭 → 자동 PnP → 무지개 격자 확인 → s 저장.

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use opencv::core::{Mat, Point, Rect, Scalar, Vec3b};
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::{
    CameraId, CameraParams, FrameSource, OpenCvCapture, PixelPickMouse, PixelPoint, Point3,
    PreviewAction, TABLE_LANDMARK_COUNT, TableLandmark, arrow_delta, calibrate_table_pnp,
    destroy_window, draw_debug_lines, draw_help_lines, draw_pixel_loupe, show_bgr,
    table_landmark_mesh_edges, table_landmarks,
};

use crate::args::{Args, pending_path, resolve_camera_id, resolve_output};
use crate::cli;
use crate::world_grid::{WorldGridParams, apply_grid_key, draw_world_grid};

struct Solved {
    params: CameraParams,
    rmse: f64,
    candidates: usize,
    /// `rmse <= max_rmse` 이면 저장 가능
    accepted: bool,
}

pub fn run(args: &Args) -> Result<()> {
    let cam_id = resolve_camera_id(args).map_err(anyhow::Error::msg)?;
    let resolved = args.cam.resolve_one().map_err(anyhow::Error::msg)?;

    let mut source: Box<dyn FrameSource> = if let Some(path) = &args.path {
        Box::new(
            OpenCvCapture::from_path(cam_id, path)
                .map_err(anyhow::Error::msg)
                .context("path")?,
        )
    } else {
        let (_r, src) = args.cam.open_one().map_err(anyhow::Error::msg)?;
        src
    };

    let window = "calib:table-pnp";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    let mouse: Arc<Mutex<PixelPickMouse>> = Arc::new(Mutex::new(PixelPickMouse::default()));
    {
        let mouse = Arc::clone(&mouse);
        highgui::set_mouse_callback(
            window,
            Some(Box::new(move |event, x, y, flags| {
                if let Ok(mut m) = mouse.lock() {
                    m.on_event(event, x, y, flags);
                }
            })),
        )?;
    }

    let marks = table_landmarks();
    let mut frozen = false;
    let mut freeze_img: Option<Mat> = None;
    let mut clicks: Vec<PixelPoint> = Vec::new();
    let mut solved: Option<Solved> = None;
    let mut grid = WorldGridParams::default();
    let mut last_fail_rmse: Option<f64> = None;
    let mut display_scale = 1.0;

    println!(
        "table-PnP — role={} cam_id={} device={} backend={} fov_y={} max_rmse={} pad={}",
        resolved.role,
        cam_id.0,
        resolved.device,
        args.cam.stream.backend,
        args.fov_y,
        args.max_rmse,
        args.pad
    );
    cli::hint_pending_if_exists(args, cam_id);
    println!(
        "Space=freeze  LMB/Enter=click  arrows|hjkl=1px  Shift+move=loupe  z=undo  c=clear  s=promote  n=live  q=quit"
    );
    println!(
        "(accepted → pending; s promotes → {})",
        resolve_output(args).display()
    );
    for (i, m) in marks.iter().enumerate() {
        println!("  {}: {}", i + 1, m.prompt);
    }

    loop {
        let mut clicks_changed = false;
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
            freeze_img = Some(img.try_clone().map_err(|e| anyhow::anyhow!("clone: {e}"))?);
            img
        };
        let img_w = frame_img.cols();
        let img_h = frame_img.rows();
        let pad = if frozen { args.pad.max(0) } else { 0 };
        let canvas_w = img_w + 2 * pad;
        let canvas_h = img_h + 2 * pad;

        let hover = {
            let mut m = mouse.lock().expect("mouse");
            m.sync(display_scale, canvas_w, canvas_h);
            if frozen {
                for (x, y) in m.drain_clicks() {
                    if clicks.len() < TABLE_LANDMARK_COUNT {
                        let fx = x - pad;
                        let fy = y - pad;
                        clicks.push(PixelPoint::new(f64::from(fx), f64::from(fy)));
                        clicks_changed = true;
                        println!(
                            "click {}/{} → ({fx},{fy})  {}",
                            clicks.len(),
                            TABLE_LANDMARK_COUNT,
                            marks[clicks.len() - 1].id
                        );
                    }
                }
            } else {
                m.clear_clicks();
            }
            m.hover
        };

        if clicks_changed {
            if clicks.len() < TABLE_LANDMARK_COUNT {
                solved = None;
                last_fail_rmse = None;
            } else {
                try_solve(
                    args,
                    cam_id,
                    freeze_img.as_ref().expect("freeze_img"),
                    &clicks,
                    &mut solved,
                    &mut last_fail_rmse,
                )?;
            }
        }

        let mut panel = if frozen {
            make_padded_canvas(&frame_img, pad)?
        } else {
            frame_img
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?
        };
        // loupe 샘플용 (오버레이 없는 패딩 캔버스)
        let loupe_src = if frozen && pad > 0 {
            Some(panel.try_clone().map_err(|e| anyhow::anyhow!("clone: {e}"))?)
        } else {
            None
        };

        if frozen {
            if let Some(ref s) = solved {
                if s.accepted {
                    if pad > 0 {
                        let mut grid_layer = frame_img
                            .try_clone()
                            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
                        draw_world_grid(&mut grid_layer, &s.params, grid)?;
                        let roi = Rect::new(pad, pad, img_w, img_h);
                        {
                            let mut dst = Mat::roi_mut(&mut panel, roi)?;
                            grid_layer.copy_to(&mut dst)?;
                        }
                    } else {
                        draw_world_grid(&mut panel, &s.params, grid)?;
                    }
                }
                draw_reproj_overlay(&mut panel, &clicks, &marks, &s.params, pad)?;
            }
            draw_clicks(&mut panel, &clicks, &marks, pad)?;

            if let Some(ref s) = solved {
                if s.accepted {
                    let lines = [
                        format!("SOLVED rmse={:.2}px — pending saved, s=promote", s.rmse),
                        format!(
                            "xy={:.2} z={:.2} layers={}",
                            grid.xy_step, grid.z_step, grid.z_layers
                        ),
                    ];
                    draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 0.0, 0.0))?;
                    draw_help_lines(
                        &mut panel,
                        &[
                            "+/- xy  [] layers  ., z",
                            "arrows|hjkl 1px  Shift loupe",
                            "LMB/Enter  z/c  s promote",
                            "n live  q quit",
                        ],
                        Scalar::new(0.0, 255.0, 80.0, 0.0),
                    )?;
                } else {
                    let lines = [
                        format!(
                            "FAIL rmse={:.2} > {:.0} — green=click magenta=ideal",
                            s.rmse, args.max_rmse
                        ),
                        "pull green toward magenta (z/c) or --fov-y".to_string(),
                    ];
                    draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 128.0, 255.0, 0.0))?;
                    draw_help_lines(
                        &mut panel,
                        &[
                            "yellow = residual",
                            "arrows|hjkl 1px  Shift loupe",
                            "LMB/Enter  z/c",
                            "n live  q quit",
                        ],
                        Scalar::new(0.0, 255.0, 80.0, 0.0),
                    )?;
                }
            } else {
                let next = if clicks.len() < TABLE_LANDMARK_COUNT {
                    marks[clicks.len()].prompt.to_string()
                } else if let Some(rmse) = last_fail_rmse {
                    format!("FAIL rmse={rmse:.2} — z/c retry or --fov-y")
                } else {
                    format!("all {TABLE_LANDMARK_COUNT} — waiting PnP")
                };
                let lines = [
                    format!("REVIEW clicks={}/{}", clicks.len(), TABLE_LANDMARK_COUNT),
                    next,
                ];
                draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
                draw_help_lines(
                    &mut panel,
                    &[
                        "LMB/Enter click",
                        "arrows|hjkl 1px  Shift loupe",
                        "z undo  c clear",
                        "n live  q quit",
                    ],
                    Scalar::new(0.0, 255.0, 80.0, 0.0),
                )?;
            }

            if let Some((hx, hy)) = hover {
                let src = loupe_src.as_ref().unwrap_or(&frame_img);
                let _ = draw_pixel_loupe(&mut panel, src, hx, hy);
            }
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
        let shown = show_bgr(window, &panel, wait)?;
        display_scale = shown.scale;
        match shown.action {
            PreviewAction::Quit => {
                let pend = pending_path(args);
                if pend.is_file() {
                    println!("quit — pending kept at {}", pend.display());
                }
                break;
            }
            PreviewAction::Continue => {}
            PreviewAction::Key(k) => {
                if frozen {
                    if let Some((dx, dy)) = arrow_delta(k) {
                        let mut m = mouse.lock().expect("mouse");
                        m.sync(display_scale, canvas_w, canvas_h);
                        m.nudge(dx, dy, canvas_w, canvas_h);
                        continue;
                    }
                    if k == 13 || k == 10 {
                        mouse.lock().expect("mouse").confirm();
                        continue;
                    }
                }
                let key = k & 0xff;
                if !frozen && key == i32::from(b' ') {
                    if freeze_img.is_some() {
                        frozen = true;
                        clicks.clear();
                        solved = None;
                        last_fail_rmse = None;
                        println!("frozen — click landmarks in order");
                    }
                } else if key == i32::from(b'n') || key == i32::from(b'N') {
                    frozen = false;
                    clicks.clear();
                    solved = None;
                    last_fail_rmse = None;
                } else if key == i32::from(b'z') || key == i32::from(b'Z') {
                    clicks.pop();
                    solved = None;
                    last_fail_rmse = None;
                } else if key == i32::from(b'c') || key == i32::from(b'C') {
                    clicks.clear();
                    solved = None;
                    last_fail_rmse = None;
                } else if key == i32::from(b's') || key == i32::from(b'S') {
                    if let Some(ref s) = solved {
                        if !s.accepted {
                            println!(
                                "rmse {:.2} > {} — 저장 불가 (초록→마젠타로 맞추거나 --fov-y)",
                                s.rmse, args.max_rmse
                            );
                            continue;
                        }
                        cli::write_result(args, s.params.clone(), s.rmse, s.candidates)?;
                        break;
                    }
                    if cli::pending_has_camera(args, cam_id) {
                        cli::promote_pending(args, cam_id)?;
                        break;
                    }
                    if clicks.len() != TABLE_LANDMARK_COUNT {
                        println!(
                            "클릭 {}/{} - 모두 찍으세요 (8점 후 자동 PnP)",
                            clicks.len(),
                            TABLE_LANDMARK_COUNT
                        );
                    } else {
                        println!("PnP 미통과 — z/c로 다시 찍거나 --fov-y");
                    }
                } else if solved.as_ref().is_some_and(|s| s.accepted) {
                    apply_grid_key(&mut grid, key);
                }
            }
        }
    }

    destroy_window(window);
    return Ok(());
}

fn try_solve(
    args: &Args,
    cam_id: CameraId,
    img: &Mat,
    clicks: &[PixelPoint],
    solved: &mut Option<Solved>,
    last_fail_rmse: &mut Option<f64>,
) -> Result<()> {
    *solved = None;
    *last_fail_rmse = None;
    if clicks.len() != TABLE_LANDMARK_COUNT {
        return Ok(());
    }
    let w = img.cols().max(1) as u32;
    let h = img.rows().max(1) as u32;
    let result =
        calibrate_table_pnp(cam_id, None, w, h, args.fov_y, clicks).map_err(anyhow::Error::msg)?;
    println!(
        "PnP candidates={} rmse={:.2}px",
        result.candidates, result.reproj_rmse
    );
    let accepted = result.reproj_rmse <= args.max_rmse;
    // FAIL여도 params 보관 → 클릭 vs 이상점 오버레이
    *solved = Some(Solved {
        params: result.params,
        rmse: result.reproj_rmse,
        candidates: result.candidates,
        accepted,
    });
    if !accepted {
        println!(
            "FAIL rmse {:.2} > {} — 초록(클릭)→마젠타(이상)로 맞추거나 --fov-y",
            result.reproj_rmse, args.max_rmse
        );
        *last_fail_rmse = Some(result.reproj_rmse);
        print_per_point_residuals(clicks, &solved.as_ref().expect("solved").params);
        return Ok(());
    }
    print_per_point_residuals(clicks, &solved.as_ref().expect("solved").params);
    let s = solved.as_ref().expect("solved");
    cli::write_pending(args, s.params.clone(), s.rmse, s.candidates)?;
    println!("SOLVED — green=click magenta=ideal, s=promote to output, q=keep pending");
    return Ok(());
}

fn print_per_point_residuals(clicks: &[PixelPoint], params: &CameraParams) {
    let marks = table_landmarks();
    let mut parts = Vec::with_capacity(TABLE_LANDMARK_COUNT);
    for (i, click) in clicks.iter().enumerate() {
        let Some(ideal) = project_unclipped(params, marks[i].world) else {
            parts.push(format!("{}:?", marks[i].id));
            continue;
        };
        let du = click.x - ideal.x;
        let dv = click.y - ideal.y;
        let err = (du * du + dv * dv).sqrt();
        parts.push(format!("{}:{err:.1}", marks[i].id));
    }
    println!("  residuals[px] {}", parts.join(" "));
}

fn to_canvas(p: PixelPoint, pad: i32) -> PixelPoint {
    return PixelPoint::new(p.x + f64::from(pad), p.y + f64::from(pad));
}

fn to_canvas_pts(pts: &[PixelPoint], pad: i32) -> Vec<PixelPoint> {
    return pts.iter().copied().map(|p| to_canvas(p, pad)).collect();
}

/// Review용: 회색 체크 패딩 + 프레임. `pad==0`이면 프레임 복제.
fn make_padded_canvas(frame: &Mat, pad: i32) -> Result<Mat> {
    if pad <= 0 {
        return frame
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"));
    }
    let fw = frame.cols();
    let fh = frame.rows();
    let cw = fw + 2 * pad;
    let ch = fh + 2 * pad;
    let mut out = Mat::zeros(ch, cw, frame.typ())?.to_mat()?;
    for y in 0..ch {
        for x in 0..cw {
            let g = if (x + y) % 2 == 0 { 40u8 } else { 72u8 };
            *out.at_2d_mut::<Vec3b>(y, x)? = Vec3b::from([g, g, g]);
        }
    }
    {
        let roi = Rect::new(pad, pad, fw, fh);
        let mut dst = Mat::roi_mut(&mut out, roi)?;
        frame.copy_to(&mut dst)?;
    }
    return Ok(out);
}

fn draw_complete_edges(
    panel: &mut Mat,
    pts: &[PixelPoint],
    color: Scalar,
    thickness: i32,
) -> Result<()> {
    for i in 0..pts.len() {
        for j in (i + 1)..pts.len() {
            let a = Point::new(pts[i].x.round() as i32, pts[i].y.round() as i32);
            let b = Point::new(pts[j].x.round() as i32, pts[j].y.round() as i32);
            imgproc::line(panel, a, b, color, thickness, imgproc::LINE_AA, 0)?;
        }
    }
    return Ok(());
}

fn draw_mesh_edges(
    panel: &mut Mat,
    pts: &[PixelPoint],
    color: Scalar,
    thickness: i32,
) -> Result<()> {
    for &(a_i, b_i) in table_landmark_mesh_edges() {
        if a_i >= pts.len() || b_i >= pts.len() {
            continue;
        }
        let a = Point::new(pts[a_i].x.round() as i32, pts[a_i].y.round() as i32);
        let b = Point::new(pts[b_i].x.round() as i32, pts[b_i].y.round() as i32);
        imgproc::line(panel, a, b, color, thickness, imgproc::LINE_AA, 0)?;
    }
    return Ok(());
}

/// 클릭 점(녹색) + 현재 꼭짓점 완전연결 메시(주황). `pad`는 캔버스 오프셋.
fn draw_clicks(
    panel: &mut Mat,
    clicks: &[PixelPoint],
    marks: &[TableLandmark],
    pad: i32,
) -> Result<()> {
    let pts = to_canvas_pts(clicks, pad);
    draw_complete_edges(panel, &pts, Scalar::new(255.0, 128.0, 0.0, 0.0), 1)?;

    for (i, px) in pts.iter().enumerate() {
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

/// PnP 해의 이상 재투영(마젠타) + 클릭↔이상 잔차(노랑) + 이상 메시.
fn draw_reproj_overlay(
    panel: &mut Mat,
    clicks: &[PixelPoint],
    marks: &[TableLandmark],
    params: &CameraParams,
    pad: i32,
) -> Result<()> {
    if clicks.len() != TABLE_LANDMARK_COUNT {
        return Ok(());
    }
    let ideals: Vec<Option<PixelPoint>> = marks
        .iter()
        .map(|m| project_unclipped(params, m.world).map(|p| to_canvas(p, pad)))
        .collect();
    let click_pts = to_canvas_pts(clicks, pad);
    let Some(ideal_pts): Option<Vec<PixelPoint>> = ideals.iter().cloned().collect() else {
        draw_residuals_partial(panel, &click_pts, &ideals)?;
        return Ok(());
    };

    draw_mesh_edges(panel, &ideal_pts, Scalar::new(255.0, 0.0, 255.0, 0.0), 2)?;
    draw_residuals_partial(panel, &click_pts, &ideals)?;
    return Ok(());
}

fn project_unclipped(params: &CameraParams, point: Point3) -> Option<PixelPoint> {
    let x_cam = params.rotation * point.coords + params.translation;
    if x_cam.z <= 0.05 {
        return None;
    }
    let u = params.fx * (x_cam.x / x_cam.z) + params.cx;
    let v = params.fy * (x_cam.y / x_cam.z) + params.cy;
    return Some(PixelPoint::new(u, v));
}

fn draw_residuals_partial(
    panel: &mut Mat,
    clicks: &[PixelPoint],
    ideals: &[Option<PixelPoint>],
) -> Result<()> {
    let residual = Scalar::new(0.0, 255.0, 255.0, 0.0); // yellow
    let ideal_pt = Scalar::new(255.0, 0.0, 255.0, 0.0); // magenta
    for (i, click) in clicks.iter().enumerate() {
        let Some(ideal) = ideals[i] else {
            continue;
        };
        let c = Point::new(click.x.round() as i32, click.y.round() as i32);
        let p = Point::new(ideal.x.round() as i32, ideal.y.round() as i32);
        imgproc::line(panel, c, p, residual, 1, imgproc::LINE_AA, 0)?;
        imgproc::draw_marker(
            panel,
            p,
            ideal_pt,
            imgproc::MARKER_CROSS,
            14,
            2,
            imgproc::LINE_AA,
        )?;
        let du = click.x - ideal.x;
        let dv = click.y - ideal.y;
        let err = (du * du + dv * dv).sqrt();
        if err >= 1.5 {
            imgproc::put_text(
                panel,
                &format!("{err:.1}"),
                Point::new(p.x + 8, p.y + 14),
                imgproc::FONT_HERSHEY_SIMPLEX,
                0.4,
                residual,
                1,
                imgproc::LINE_AA,
                false,
            )?;
        }
    }
    return Ok(());
}
