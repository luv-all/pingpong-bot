//! 라이브: Space 스냅 → 8점 클릭 → 자동 PnP → 점 미세조정 → s 저장.
//!
//! 8점이 다 모이면 **지우지 않고** 개별 점을 골라 1px씩 옮기며 재-PnP한다
//! ([`crate::adjust`]). `f`/`F`로 `fov_y`를 흔들면 클릭을 그대로 두고
//! 마젠타 평면의 기울기만 바뀐다 — 8점이 동일평면이라 RMSE로는 focal이
//! 거의 관측되지 않기 때문.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use opencv::core::{Mat, Rect, Scalar};
use opencv::highgui;
use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::camera::Landmark;
use pingpong_bot::camera::{
    FrameSource, OpenCvCapture, PixelPickMouse, Preview, PreviewAction, TablePnp, WorldGridParams,
};
use pingpong_bot::constants::TABLE_LANDMARK_COUNT;

use crate::adjust::{self, Adjust};
use crate::args::{Args, pending_path, resolve_camera_id, resolve_output};
use crate::cli;
use crate::overlay;

/// 마지막 조정 후 이만큼 조용해지면 잔차 출력 + pending 저장.
/// 키 하나마다 JSON을 쓰지 않으려는 debounce.
const SETTLE: Duration = Duration::from_millis(400);

struct Solved {
    params: camera::Params,
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

    let marks = TablePnp::landmarks();
    let mut frozen = false;
    let mut freeze_img: Option<Mat> = None;
    let mut clicks: Vec<camera::Pixel> = Vec::new();
    let mut solved: Option<Solved> = None;
    let mut grid = WorldGridParams::default();
    let mut solve_err: Option<String> = None;
    let mut display_scale = 1.0;
    let baseline = cli::load_baseline_params(args, cam_id);

    let mut adj = Adjust::new(args.fov_y);
    let mut best_rmse: Option<f64> = None;
    let mut dirty_since: Option<Instant> = None;

    println!(
        "table-PnP — role={} cam_id={} device={} backend={} fov_y={} max_rmse={} pad={} refine_radius={}",
        resolved.role,
        cam_id.0,
        resolved.device,
        args.cam.stream.backend,
        args.fov_y,
        args.max_rmse,
        args.pad,
        args.refine_radius
    );
    if let Some(ref b) = baseline {
        let src = args.merge.as_ref().unwrap_or(&args.output);
        println!(
            "baseline cam{} from {} — live/freeze overlay until first click",
            b.camera_id.0,
            src.display()
        );
    } else {
        println!(
            "no baseline for cam{} in {}",
            cam_id.0,
            resolve_output(args).display()
        );
    }
    cli::hint_pending_if_exists(args, cam_id);
    println!(
        "Space=freeze  LMB/Enter=click  arrows|hjkl=1px  Shift+move=loupe  z=undo  c=clear  s=promote  n=live  q=quit"
    );
    println!(
        "8/8 adjust: 1-8|Tab=select  0=none  arrows|hjkl=move 1px  HJKL=5px  Enter=to aim  LMB=grab  u=undo  r=refine  f/F=fov_y"
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
        let bounds = adjust::pixel_bounds(img_w, img_h, pad);

        let hover = {
            let mut m = mouse.lock().expect("mouse");
            m.sync(display_scale, canvas_w, canvas_h);
            if frozen {
                for (x, y) in m.drain_clicks() {
                    let target = camera::Pixel::new(f64::from(x - pad), f64::from(y - pad));
                    if clicks.len() < TABLE_LANDMARK_COUNT {
                        clicks.push(target);
                        clicks_changed = true;
                        if clicks.len() == 1 {
                            println!("recalib — baseline overlay off");
                        }
                        println!(
                            "click {}/{} → ({},{})  {}",
                            clicks.len(),
                            TABLE_LANDMARK_COUNT,
                            target.x,
                            target.y,
                            marks[clicks.len() - 1].id
                        );
                    } else if let Some(i) = adjust::nearest_click(&clicks, target, adjust::SNAP_PX)
                    {
                        // 기존 점 근처 클릭 = 잡기. 이동은 방향키/Enter로.
                        adj.select(i);
                        println!("grab {}:{} — arrows|hjkl to move", i + 1, marks[i].id);
                    } else if let Some(i) = adj.sel {
                        adj.push_history(&clicks);
                        clicks[i] = target;
                        clicks_changed = true;
                        println!(
                            "move {}:{} → ({},{})",
                            i + 1,
                            marks[i].id,
                            target.x,
                            target.y
                        );
                    }
                }
            } else {
                m.clear_clicks();
            }
            m.hover
        };

        if clicks_changed {
            if clicks.len() == TABLE_LANDMARK_COUNT && adj.anchor.is_empty() {
                adj.set_anchor(&clicks);
            }
            resolve_now(
                cam_id,
                freeze_img.as_ref().expect("freeze_img"),
                adj.fov_y,
                args.max_rmse,
                &clicks,
                &mut solved,
                &mut solve_err,
                &mut dirty_since,
            );
        }

        if let Some(ref s) = solved {
            best_rmse = Some(best_rmse.map_or(s.rmse, |b| b.min(s.rmse)));
        }

        // 조정이 멈춘 뒤에만 stdout·pending 파일을 건드린다.
        if dirty_since.is_some_and(|t| t.elapsed() >= SETTLE) {
            dirty_since = None;
            if let Some(ref e) = solve_err {
                println!("PnP 실패: {e}");
            }
            if let Some(ref s) = solved {
                println!(
                    "PnP candidates={} rmse={:.2}px fov_y={:.1}",
                    s.candidates, s.rmse, adj.fov_y
                );
                print_per_point_residuals(&clicks, &s.params);
                if s.accepted {
                    cli::write_pending(args, s.params.clone(), s.rmse, s.candidates)?;
                    println!(
                        "SOLVED — green=click magenta=ideal, s=promote to output, q=keep pending"
                    );
                } else {
                    println!(
                        "FAIL rmse {:.2} > {} — 1-8로 점 골라 미세조정, r=refine, f/F=fov_y",
                        s.rmse, args.max_rmse
                    );
                }
            }
        }

        let mut panel = if frozen {
            overlay::make_padded_canvas(&frame_img, pad)?
        } else {
            frame_img
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?
        };
        // loupe 샘플용 (오버레이 없는 패딩 캔버스)
        let loupe_src = if frozen && pad > 0 {
            Some(
                panel
                    .try_clone()
                    .map_err(|e| anyhow::anyhow!("clone: {e}"))?,
            )
        } else {
            None
        };

        if frozen {
            let residuals = solved
                .as_ref()
                .map(|s| overlay::per_point_residuals(&clicks, &s.params))
                .unwrap_or_default();

            if let Some(ref s) = solved {
                if s.accepted {
                    overlay_world_grid(&mut panel, &frame_img, &s.params, grid, pad, img_w, img_h)?;
                }
                overlay::draw_reproj_overlay(&mut panel, &clicks, &marks, &s.params, pad, adj.sel)?;
            } else if clicks.is_empty() {
                if let Some(ref b) = baseline {
                    overlay_world_grid(&mut panel, &frame_img, b, grid, pad, img_w, img_h)?;
                }
            }
            overlay::draw_anchor_bound(&mut panel, &adj.anchor, pad, adj.sel, args.refine_radius)?;
            overlay::draw_clicks(&mut panel, &clicks, &marks, pad, adj.sel)?;

            if let Some(ref s) = solved {
                let sel_line = sel_hud_line(&adj, &clicks, &residuals, &marks);
                if s.accepted {
                    let lines = [
                        format!(
                            "SOLVED rmse={:.2}px (best {:.2}) fov_y={:.1}",
                            s.rmse,
                            best_rmse.unwrap_or(s.rmse),
                            adj.fov_y
                        ),
                        sel_line,
                        format!(
                            "xy={:.2} z={:.2} layers={}",
                            grid.xy_step, grid.z_step, grid.z_layers
                        ),
                    ];
                    Preview::draw_debug_lines(
                        &mut panel,
                        &lines,
                        Scalar::new(0.0, 255.0, 0.0, 0.0),
                    )?;
                    Preview::draw_help_lines(
                        &mut panel,
                        &adjust_help(),
                        Scalar::new(0.0, 255.0, 80.0, 0.0),
                    )?;
                } else {
                    let lines = [
                        format!(
                            "FAIL rmse={:.2} > {:.0} (best {:.2}) fov_y={:.1}",
                            s.rmse,
                            args.max_rmse,
                            best_rmse.unwrap_or(s.rmse),
                            adj.fov_y
                        ),
                        sel_line,
                        "green=click magenta=ideal yellow=residual".to_string(),
                    ];
                    Preview::draw_debug_lines(
                        &mut panel,
                        &lines,
                        Scalar::new(0.0, 128.0, 255.0, 0.0),
                    )?;
                    Preview::draw_help_lines(
                        &mut panel,
                        &adjust_help(),
                        Scalar::new(0.0, 255.0, 80.0, 0.0),
                    )?;
                }
            } else if clicks.is_empty() && baseline.is_some() {
                let lines = [
                    format!("EXISTING cam{} — click to recalibrate", cam_id.0),
                    format!(
                        "xy={:.2} z={:.2} layers={}",
                        grid.xy_step, grid.z_step, grid.z_layers
                    ),
                ];
                Preview::draw_debug_lines(&mut panel, &lines, Scalar::new(255.0, 128.0, 0.0, 0.0))?;
                Preview::draw_help_lines(
                    &mut panel,
                    &[
                        "+/- xy  [] layers  ., z",
                        "LMB/Enter start recalib",
                        "arrows|hjkl 1px  Shift loupe",
                        "n live  q quit",
                    ],
                    Scalar::new(0.0, 255.0, 80.0, 0.0),
                )?;
            } else {
                let next = if clicks.len() < TABLE_LANDMARK_COUNT {
                    marks[clicks.len()].prompt.to_string()
                } else if let Some(ref e) = solve_err {
                    format!("PnP failed: {e}")
                } else {
                    format!("all {TABLE_LANDMARK_COUNT} — waiting PnP")
                };
                let lines = [
                    format!("REVIEW clicks={}/{}", clicks.len(), TABLE_LANDMARK_COUNT),
                    next,
                ];
                Preview::draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
                Preview::draw_help_lines(
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

            // 조정 중이면 loupe를 선택된 점에 붙인다 (마우스 위치보다 유용).
            let loupe_at = match adj.sel.filter(|_| clicks.len() == TABLE_LANDMARK_COUNT) {
                Some(i) => {
                    let c = overlay::to_canvas(clicks[i], pad);
                    Some((c.x.round() as i32, c.y.round() as i32))
                }
                None => hover,
            };
            if let Some((hx, hy)) = loupe_at {
                let src = loupe_src.as_ref().unwrap_or(&frame_img);
                let _ = Preview::draw_pixel_loupe(&mut panel, src, hx, hy);
            }
        } else {
            if let Some(ref b) = baseline {
                Preview::draw_world_grid(&mut panel, b, &grid)?;
                let lines = [
                    format!("LIVE — existing cam{} overlay", cam_id.0),
                    "Space freeze · first click starts recalib".into(),
                ];
                Preview::draw_debug_lines(&mut panel, &lines, Scalar::new(255.0, 128.0, 0.0, 0.0))?;
                Preview::draw_help_lines(
                    &mut panel,
                    &["+/- [] ., grid", "Space freeze", "q quit"],
                    Scalar::new(0.0, 255.0, 80.0, 0.0),
                )?;
            } else {
                Preview::draw_debug_lines(
                    &mut panel,
                    &["LIVE - Space to freeze"],
                    Scalar::new(0.0, 255.0, 255.0, 0.0),
                )?;
                Preview::draw_help_lines(
                    &mut panel,
                    &["Space freeze", "q quit"],
                    Scalar::new(0.0, 255.0, 80.0, 0.0),
                )?;
            }
        }

        let wait = if frozen { 30 } else { 1 };
        let shown = Preview::show_bgr(window, &panel, wait)?;
        display_scale = shown.scale;
        match shown.action {
            PreviewAction::Quit => {
                let pend = pending_path(args);
                if pend.is_file() {
                    println!("quit — pending kept at {}", pend.display());
                }
                print_fov_hint(&adj, args);
                break;
            }
            PreviewAction::Continue => {}
            PreviewAction::Key(k) => {
                let key = k & 0xff;
                let adjusting = frozen && clicks.len() == TABLE_LANDMARK_COUNT;

                if frozen {
                    // 대문자 HJKL을 arrow_delta보다 먼저 — arrow_delta는 대소문자를 같게 본다.
                    if let (Some(i), Some((dx, dy))) =
                        (adj.sel.filter(|_| adjusting), coarse_delta(key))
                    {
                        if shift_selected(
                            &mut clicks,
                            &mut adj,
                            i,
                            dx * adjust::COARSE_STEP,
                            dy * adjust::COARSE_STEP,
                            bounds,
                        ) {
                            resolve_now(
                                cam_id,
                                freeze_img.as_ref().expect("freeze_img"),
                                adj.fov_y,
                                args.max_rmse,
                                &clicks,
                                &mut solved,
                                &mut solve_err,
                                &mut dirty_since,
                            );
                        }
                        continue;
                    }
                    if let Some((dx, dy)) = Preview::arrow_delta(k) {
                        if let Some(i) = adj.sel.filter(|_| adjusting) {
                            if shift_selected(
                                &mut clicks,
                                &mut adj,
                                i,
                                f64::from(dx),
                                f64::from(dy),
                                bounds,
                            ) {
                                resolve_now(
                                    cam_id,
                                    freeze_img.as_ref().expect("freeze_img"),
                                    adj.fov_y,
                                    args.max_rmse,
                                    &clicks,
                                    &mut solved,
                                    &mut solve_err,
                                    &mut dirty_since,
                                );
                            }
                        } else {
                            let mut m = mouse.lock().expect("mouse");
                            m.sync(display_scale, canvas_w, canvas_h);
                            m.nudge(dx, dy, canvas_w, canvas_h);
                        }
                        continue;
                    }
                    if k == 13 || k == 10 {
                        mouse.lock().expect("mouse").confirm();
                        continue;
                    }
                    // Tab — 선택 순회 (방향키는 위에서 이미 소비됨)
                    if key == 9 && adjusting {
                        adj.cycle();
                        if let Some(i) = adj.sel {
                            println!("select {}:{}", i + 1, marks[i].id);
                        }
                        continue;
                    }
                    // 1-8 선택 / 0 해제
                    if adjusting && (i32::from(b'0')..=i32::from(b'8')).contains(&key) {
                        let d = (key - i32::from(b'0')) as usize;
                        if d == 0 {
                            adj.clear_sel();
                            println!("select none — arrows move aim again");
                        } else {
                            adj.select(d - 1);
                            println!("select {}:{}", d, marks[d - 1].id);
                        }
                        continue;
                    }
                }

                if !frozen && key == i32::from(b' ') {
                    if freeze_img.is_some() {
                        frozen = true;
                        clicks.clear();
                        solved = None;
                        solve_err = None;
                        adj.reset(args.fov_y);
                        best_rmse = None;
                        dirty_since = None;
                        println!("frozen — click landmarks in order");
                    }
                } else if key == i32::from(b'n') || key == i32::from(b'N') {
                    frozen = false;
                    clicks.clear();
                    solved = None;
                    solve_err = None;
                    adj.reset(args.fov_y);
                    best_rmse = None;
                    dirty_since = None;
                } else if key == i32::from(b'z') || key == i32::from(b'Z') {
                    clicks.pop();
                    solved = None;
                    solve_err = None;
                    // fov_y는 유지 — 튜닝한 값을 클릭 하나 때문에 잃지 않게.
                    adj.exit_adjust();
                    best_rmse = None;
                    dirty_since = None;
                } else if key == i32::from(b'c') || key == i32::from(b'C') {
                    clicks.clear();
                    solved = None;
                    solve_err = None;
                    adj.reset(args.fov_y);
                    best_rmse = None;
                    dirty_since = None;
                } else if key == i32::from(b'u') || key == i32::from(b'U') {
                    if let Some(snap) = adj.undo() {
                        clicks = snap.clicks;
                        println!("undo — {} left, fov_y={:.1}", adj.history_len(), adj.fov_y);
                        resolve_now(
                            cam_id,
                            freeze_img.as_ref().expect("freeze_img"),
                            adj.fov_y,
                            args.max_rmse,
                            &clicks,
                            &mut solved,
                            &mut solve_err,
                            &mut dirty_since,
                        );
                    } else {
                        println!("undo — 되돌릴 조정 없음");
                    }
                } else if adjusting && (key == i32::from(b'f') || key == i32::from(b'F')) {
                    let delta = if key == i32::from(b'F') {
                        adjust::FOV_STEP_DEG
                    } else {
                        -adjust::FOV_STEP_DEG
                    };
                    adj.push_history(&clicks);
                    if adj.nudge_fov(delta) {
                        resolve_now(
                            cam_id,
                            freeze_img.as_ref().expect("freeze_img"),
                            adj.fov_y,
                            args.max_rmse,
                            &clicks,
                            &mut solved,
                            &mut solve_err,
                            &mut dirty_since,
                        );
                    } else {
                        // 한계에서 막혔으면 쓸데없는 undo 항목을 남기지 않는다.
                        let _ = adj.undo();
                    }
                } else if adjusting && (key == i32::from(b'r') || key == i32::from(b'R')) {
                    run_refine(
                        args,
                        cam_id,
                        freeze_img.as_ref().expect("freeze_img"),
                        &marks,
                        bounds,
                        &mut clicks,
                        &mut adj,
                        &mut solved,
                        &mut solve_err,
                        &mut dirty_since,
                    );
                } else if key == i32::from(b's') || key == i32::from(b'S') {
                    if let Some(ref s) = solved {
                        if !s.accepted {
                            println!(
                                "rmse {:.2} > {} — 저장 불가 (1-8로 점 미세조정, r=refine, f/F=fov_y)",
                                s.rmse, args.max_rmse
                            );
                            continue;
                        }
                        cli::write_result(args, s.params.clone(), s.rmse, s.candidates)?;
                        print_fov_hint(&adj, args);
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
                        println!("PnP 미통과 — 1-8로 점 미세조정하거나 f/F로 fov_y");
                    }
                } else if solved.as_ref().is_some_and(|s| s.accepted)
                    || (baseline.is_some() && clicks.is_empty())
                {
                    Preview::apply_grid_key(&mut grid, key);
                }
            }
        }
    }

    Preview::destroy_window(window);
    return Ok(());
}

/// 대문자 `HJKL` → 단위 방향. 소문자·방향키는 `Preview::arrow_delta`가 본다.
///
/// `wait_key_ex`의 Shift 수정자 비트는 백엔드마다 달라 못 믿으므로 대문자로 구분한다.
fn coarse_delta(key: i32) -> Option<(f64, f64)> {
    return match key {
        k if k == i32::from(b'H') => Some((-1.0, 0.0)),
        k if k == i32::from(b'L') => Some((1.0, 0.0)),
        k if k == i32::from(b'K') => Some((0.0, -1.0)),
        k if k == i32::from(b'J') => Some((0.0, 1.0)),
        _ => None,
    };
}

/// 선택된 점을 옮긴다 (되돌릴 지점 기록). 경계에 걸려 안 움직이면 false.
fn shift_selected(
    clicks: &mut [camera::Pixel],
    adj: &mut Adjust,
    index: usize,
    dx: f64,
    dy: f64,
    bounds: adjust::PixelBounds,
) -> bool {
    let Some(&current) = clicks.get(index) else {
        return false;
    };
    let next = adjust::moved_point(current, dx, dy, bounds);
    if next == current {
        return false;
    }
    adj.push_history(clicks);
    clicks[index] = next;
    return true;
}

#[allow(clippy::too_many_arguments)]
fn run_refine(
    args: &Args,
    cam_id: camera::Id,
    img: &Mat,
    marks: &[Landmark],
    bounds: adjust::PixelBounds,
    clicks: &mut Vec<camera::Pixel>,
    adj: &mut Adjust,
    solved: &mut Option<Solved>,
    solve_err: &mut Option<String>,
    dirty_since: &mut Option<Instant>,
) {
    if adj.anchor.len() != TABLE_LANDMARK_COUNT {
        println!("refine — 8점을 먼저 다 찍으세요");
        return;
    }
    let w = img.cols().max(1) as u32;
    let h = img.rows().max(1) as u32;
    let Some(out) = adjust::refine_clicks(
        cam_id,
        w,
        h,
        adj.fov_y,
        &adj.anchor,
        clicks,
        args.refine_radius,
        bounds,
    ) else {
        println!("refine — PnP 해가 없어 건너뜀");
        return;
    };

    let mut moved = Vec::new();
    for (i, p) in out.clicks.iter().enumerate() {
        let dx = p.x - clicks[i].x;
        let dy = p.y - clicks[i].y;
        if dx.abs() > 1e-9 || dy.abs() > 1e-9 {
            moved.push(format!("{}:{dx:+.1},{dy:+.1}", marks[i].id));
        }
    }
    println!(
        "refine rmse {:.2} → {:.2} px (radius={:.1}px, solves={})",
        out.rmse_before, out.rmse_after, args.refine_radius, out.solves
    );
    println!(
        "  moved {}",
        if moved.is_empty() {
            "(none)".to_string()
        } else {
            moved.join(" ")
        }
    );
    if moved.is_empty() {
        return;
    }

    adj.push_history(clicks);
    *clicks = out.clicks;
    resolve_now(
        cam_id,
        img,
        adj.fov_y,
        args.max_rmse,
        clicks,
        solved,
        solve_err,
        dirty_since,
    );
}

/// 조정 직후 즉시 재-PnP. 출력·파일 IO는 하지 않고 settle 타이머만 다시 잡는다.
#[allow(clippy::too_many_arguments)]
fn resolve_now(
    cam_id: camera::Id,
    img: &Mat,
    fov_y: f64,
    max_rmse: f64,
    clicks: &[camera::Pixel],
    solved: &mut Option<Solved>,
    solve_err: &mut Option<String>,
    dirty_since: &mut Option<Instant>,
) {
    if clicks.len() != TABLE_LANDMARK_COUNT {
        *solved = None;
        *solve_err = None;
        *dirty_since = None;
        return;
    }
    match solve_quiet(cam_id, img, fov_y, max_rmse, clicks) {
        Ok(s) => {
            *solved = Some(s);
            *solve_err = None;
        }
        Err(e) => {
            *solved = None;
            *solve_err = Some(e);
        }
    }
    *dirty_since = Some(Instant::now());
}

/// PnP 한 번. FAIL이어도 params를 들고 온다 (클릭 vs 이상점 오버레이용).
fn solve_quiet(
    cam_id: camera::Id,
    img: &Mat,
    fov_y: f64,
    max_rmse: f64,
    clicks: &[camera::Pixel],
) -> Result<Solved, String> {
    let w = img.cols().max(1) as u32;
    let h = img.rows().max(1) as u32;
    let result = TablePnp::calibrate(cam_id, None, w, h, fov_y, clicks)?;
    let accepted = result.reproj_rmse <= max_rmse;
    return Ok(Solved {
        params: result.params,
        rmse: result.reproj_rmse,
        candidates: result.candidates,
        accepted,
    });
}

fn adjust_help() -> [&'static str; 5] {
    return [
        "1-8|Tab select  0 none  u undo",
        "arrows|hjkl 1px  HJKL 5px",
        "Enter to aim  LMB grab  r refine",
        "f/F fov_y  +/- [] ., grid",
        "s promote  n live  q quit",
    ];
}

fn sel_hud_line(
    adj: &Adjust,
    clicks: &[camera::Pixel],
    residuals: &[Option<f64>],
    marks: &[Landmark],
) -> String {
    let Some(i) = adj.sel else {
        return "SEL none — 1-8|Tab to pick a point".to_string();
    };
    let res = residuals
        .get(i)
        .copied()
        .flatten()
        .map_or_else(|| "?".to_string(), |r| format!("{r:.1}px"));
    let (dx, dy) = adj.offset_from_anchor(clicks).unwrap_or((0.0, 0.0));
    return format!(
        "SEL {}:{} res={res} d=({dx:+.1},{dy:+.1})",
        i + 1,
        marks[i].id
    );
}

/// `f`/`F`로 옮긴 값을 다음 실행에 쓸 수 있게 알려준다.
fn print_fov_hint(adj: &Adjust, args: &Args) {
    if (adj.fov_y - args.fov_y).abs() > 1e-9 {
        println!(
            "fov_y tuned {:.1} → {:.1} — rerun with --fov-y {:.1}",
            args.fov_y, adj.fov_y, adj.fov_y
        );
    }
}

/// 패딩 캔버스면 원본 ROI에만 격자, 아니면 panel 전체에.
fn overlay_world_grid(
    panel: &mut Mat,
    frame_img: &Mat,
    params: &camera::Params,
    grid: WorldGridParams,
    pad: i32,
    img_w: i32,
    img_h: i32,
) -> Result<()> {
    if pad > 0 {
        let mut grid_layer = frame_img
            .try_clone()
            .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
        Preview::draw_world_grid(&mut grid_layer, params, &grid)?;
        let roi = Rect::new(pad, pad, img_w, img_h);
        let mut dst = Mat::roi_mut(panel, roi)?;
        grid_layer.copy_to(&mut dst)?;
    } else {
        Preview::draw_world_grid(panel, params, &grid)?;
    }
    return Ok(());
}

fn print_per_point_residuals(clicks: &[camera::Pixel], params: &camera::Params) {
    let marks = TablePnp::landmarks();
    let parts: Vec<String> = overlay::per_point_residuals(clicks, params)
        .iter()
        .enumerate()
        .map(|(i, r)| match r {
            Some(err) => format!("{}:{err:.1}", marks[i].id),
            None => format!("{}:?", marks[i].id),
        })
        .collect();
    println!("  residuals[px] {}", parts.join(" "));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 키 디스패치가 `coarse_delta`를 `arrow_delta`보다 **먼저** 봐야 하는 이유.
    /// `arrow_delta`가 대소문자를 같게 보므로 순서가 뒤바뀌면 `HJKL`이 1px로 먹힌다.
    #[test]
    fn arrow_delta_cannot_distinguish_case() {
        assert!(Preview::arrow_delta(i32::from(b'H')).is_some());
        assert!(Preview::arrow_delta(i32::from(b'h')).is_some());
        assert_eq!(
            Preview::arrow_delta(i32::from(b'H')),
            Preview::arrow_delta(i32::from(b'h'))
        );
    }

    #[test]
    fn coarse_delta_only_matches_uppercase_hjkl() {
        assert_eq!(coarse_delta(i32::from(b'H')), Some((-1.0, 0.0)));
        assert_eq!(coarse_delta(i32::from(b'L')), Some((1.0, 0.0)));
        assert_eq!(coarse_delta(i32::from(b'K')), Some((0.0, -1.0)));
        assert_eq!(coarse_delta(i32::from(b'J')), Some((0.0, 1.0)));
        // 소문자는 1px 경로(arrow_delta)로 흘러가야 한다
        for c in [b'h', b'j', b'k', b'l'] {
            assert_eq!(coarse_delta(i32::from(c)), None, "{}", c as char);
        }
        // 다른 단축키와 겹치지 않는다
        for c in [b's', b'n', b'z', b'c', b'u', b'r', b'f', b'F', b'0', b'8'] {
            assert_eq!(coarse_delta(i32::from(c)), None, "{}", c as char);
        }
    }

    /// macOS/X11/Win32 방향키 코드는 `k & 0xff`가 Tab(9)이나 `HJKL`과 겹치지 않는다.
    #[test]
    fn arrow_key_codes_do_not_alias_letter_shortcuts() {
        let arrows = [
            0xF700,
            0xF701,
            0xF702,
            0xF703, // Cocoa
            0xFF51,
            0xFF52,
            0xFF53,
            0xFF54, // X11
            0x25 << 16,
            0x26 << 16,
            0x27 << 16,
            0x28 << 16, // Win32 VK
        ];
        for k in arrows {
            let key = k & 0xff;
            assert_ne!(key, 9, "arrow {k:#x} aliases Tab");
            assert_eq!(coarse_delta(key), None, "arrow {k:#x} aliases HJKL");
        }
    }

    #[test]
    fn shift_selected_records_history_and_stops_at_bounds() {
        let bounds = adjust::pixel_bounds(640, 480, 0);
        let mut clicks = vec![camera::Pixel::new(5.0, 5.0)];
        let mut adj = Adjust::new(47.3);

        assert!(shift_selected(&mut clicks, &mut adj, 0, -1.0, 0.0, bounds));
        assert_eq!(clicks[0], camera::Pixel::new(4.0, 5.0));
        assert_eq!(adj.history_len(), 1);

        // 경계에 붙으면 더 안 움직이고 히스토리도 늘지 않는다
        clicks[0] = camera::Pixel::new(0.0, 0.0);
        assert!(!shift_selected(&mut clicks, &mut adj, 0, -1.0, 0.0, bounds));
        assert_eq!(adj.history_len(), 1);

        // 없는 인덱스는 no-op
        assert!(!shift_selected(&mut clicks, &mut adj, 7, 1.0, 0.0, bounds));
        assert_eq!(adj.history_len(), 1);
    }

    #[test]
    fn sel_hud_line_reports_residual_and_anchor_offset() {
        let marks = TablePnp::landmarks();
        let mut adj = Adjust::new(47.3);
        let clicks = vec![camera::Pixel::new(12.0, 9.0)];

        assert!(sel_hud_line(&adj, &clicks, &[], &marks).starts_with("SEL none"));

        adj.set_anchor(&[camera::Pixel::new(10.0, 10.0)]);
        adj.select(0);
        let line = sel_hud_line(&adj, &clicks, &[Some(4.25)], &marks);
        assert!(line.contains("1:c00"), "{line}");
        assert!(line.contains("res=4.2px"), "{line}");
        assert!(line.contains("d=(+2.0,-1.0)"), "{line}");

        // 재투영이 카메라 뒤로 가면 '?'
        let line = sel_hud_line(&adj, &clicks, &[None], &marks);
        assert!(line.contains("res=?"), "{line}");
    }
}
