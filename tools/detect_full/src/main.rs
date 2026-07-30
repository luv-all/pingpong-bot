//! Detector 본선 디버그 — floor-edge 마스크 + adaptive ROI + 누적 파이프라인 패널.
//!
//! 스텝: `0 raw → 1 floor-mask → 2 colormask → 3 +contour → 4 roi`
//! track 중이면 2·3은 ROI 크롭에서만 계산(본선과 동일).
//! 키: `r` ROI · `[` `]` radius_scale · `,` `.` motion_scale · `-` `=` padding · `p` paste · `q`/ESC

mod cli;

use anyhow::{Context, Result, bail};
use clap::Parser;
use opencv::core::{Rect, Scalar, Vector};
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::camera::{Frame, FrameSource, ImageDirSource, Preview, PreviewAction};
use pingpong_bot::defaults::{colormask_for, detector_for};
use pingpong_bot::detector::{
    AppearanceChain, ColormaskDetector, ContourDetector, RoiTrack, Scorer,
};

use cli::Args;

fn open_source(args: &Args) -> Result<Box<dyn FrameSource>> {
    let cam_id = args.cam.camera_id().map_err(anyhow::Error::msg)?;
    if let Some(images) = &args.images {
        if args.offline.has_offline() {
            bail!("--images 와 --clip 동시 사용 불가");
        }
        return Ok(Box::new(
            ImageDirSource::open(cam_id, images)
                .map_err(anyhow::Error::msg)
                .context("images")?,
        ));
    }
    return Ok(args
        .cam
        .open_mono_input(&args.offline)
        .map_err(anyhow::Error::msg)?);
}

fn vstack_bgr(top: &Mat, bottom: &Mat) -> Result<Mat> {
    let w = top.cols().max(bottom.cols()).max(1);
    let pad = |m: &Mat| -> Result<Mat> {
        if m.cols() == w {
            return Ok(m.try_clone()?);
        }
        let mut canvas = Mat::zeros(m.rows(), w, m.typ())?.to_mat()?;
        let roi = Rect::new(0, 0, m.cols(), m.rows());
        let mut dst = Mat::roi_mut(&mut canvas, roi)?;
        m.copy_to(&mut dst)?;
        return Ok(canvas);
    };
    let a = pad(top)?;
    let b = pad(bottom)?;
    let mut out = Mat::default();
    opencv::core::vconcat(&Vector::<Mat>::from_iter([a, b]), &mut out)?;
    return Ok(out);
}

fn empty_like(frame: &Frame) -> Result<Mat> {
    return Ok(Mat::zeros(frame.image.rows(), frame.image.cols(), frame.image.typ())?.to_mat()?);
}

fn paste_at(dst: &mut Mat, src: &Mat, r: Rect) -> Result<()> {
    if src.cols() != r.width || src.rows() != r.height {
        return Ok(());
    }
    let mut view = Mat::roi_mut(dst, r)?;
    src.copy_to(&mut view)?;
    return Ok(());
}

/// 본선과 같은 영역에서 appearance 스텝을 돌린다. ROI track이면 크롭.
fn appearance_steps(
    appearance: &mut AppearanceChain,
    scorer: &Scorer,
    frame: &Frame,
    roi: Option<Rect>,
) -> Result<(Option<camera::Pixel>, Mat, Mat)> {
    let Some(r) = roi else {
        let (px, cm, cas) = appearance.detect_debug(frame, scorer);
        return Ok((px, cm, cas));
    };

    let view = Mat::roi(&frame.image, r).map_err(|e| anyhow::anyhow!("roi view: {e}"))?;
    let owned = view
        .try_clone()
        .map_err(|e| anyhow::anyhow!("roi clone: {e}"))?;
    let local = Frame {
        camera_id: frame.camera_id,
        image: owned,
        timestamp: frame.timestamp,
    };
    let (local_px, cm_local, cas_local) = appearance.detect_debug(&local, scorer);

    let mut cm_full = empty_like(frame)?;
    let mut cas_full = empty_like(frame)?;
    paste_at(&mut cm_full, &cm_local, r)?;
    paste_at(&mut cas_full, &cas_local, r)?;

    let px = local_px.map(|p| camera::Pixel::new(p.x + f64::from(r.x), p.y + f64::from(r.y)));
    return Ok((px, cm_full, cas_full));
}

fn nonzero_bgr(bgr: &Mat) -> i32 {
    let mut gray = Mat::default();
    if imgproc::cvt_color(
        bgr,
        &mut gray,
        imgproc::COLOR_BGR2GRAY,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )
    .is_err()
    {
        return 0;
    }
    return opencv::core::count_non_zero(&gray).unwrap_or(0);
}

fn draw_panel_hud(img: &mut Mat, lines: &[impl AsRef<str>], color: Scalar) -> Result<()> {
    Preview::draw_debug_lines(img, lines, color).map_err(Into::into)
}

fn pixel_hud_line(label: &str, pixel: Option<camera::Pixel>, equivalent_radius: f64) -> String {
    return match pixel {
        Some(p) => format!(
            "{label}  pixel=({:.1},{:.1})  radius~{:.0}",
            p.x, p.y, equivalent_radius
        ),
        None => format!("{label}  miss"),
    };
}

fn handle_tune_key(detector: &mut RoiTrack, key: i32) -> bool {
    let p = &mut detector.params;
    let handled = match key {
        k if k == i32::from(b'[') => {
            p.radius_scale = (p.radius_scale - 0.25).max(0.0);
            true
        }
        k if k == i32::from(b']') => {
            p.radius_scale += 0.25;
            true
        }
        k if k == i32::from(b',') => {
            p.motion_scale = (p.motion_scale - 0.25).max(0.0);
            true
        }
        k if k == i32::from(b'.') => {
            p.motion_scale += 0.25;
            true
        }
        k if k == i32::from(b'-') => {
            p.padding = (p.padding - 4).max(0);
            true
        }
        k if k == i32::from(b'=') => {
            p.padding += 4;
            true
        }
        k if k == i32::from(b'p') || k == i32::from(b'P') => {
            println!(
                "// paste into RoiParams::default()\n{}",
                p.to_defaults_snippet()
            );
            false
        }
        _ => false,
    };
    if handled {
        detector.recompute_half();
        println!("{detector}");
    }
    return handled;
}

fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(dir) = &args.output {
        std::fs::create_dir_all(dir).ok();
    }

    let mut source = open_source(&args)?;
    let cam_id = source.camera_id();
    let mut detector = detector_for(cam_id)?;
    if args.no_roi {
        detector.set_roi_enabled(false);
    }

    let scorer_params = detector.scorer.clone();
    let scorer = Scorer::from(&scorer_params);
    let mut appearance = AppearanceChain::new()
        .then(ColormaskDetector::new(colormask_for(cam_id)?))
        .then(ContourDetector::from(&scorer_params));

    println!(
        "{detector} (cam{} raw → mask → color → contour → ROI) area=[{:.0},{:.0}]",
        cam_id.0, scorer_params.min_area_px, scorer_params.max_area_px
    );
    println!("keys: r ROI  [ ] radius_scale  , . motion_scale  - = padding  p paste  q/ESC quit");

    let window = "detect:full";
    let wait_ms = args
        .wait_ms
        .unwrap_or(if args.offline.has_offline() || args.images.is_some() {
            33
        } else {
            1
        });
    let preview = !args.no_preview;

    let mut n = 0usize;
    let mut hits = 0usize;
    let mut last_pixel: Option<camera::Pixel> = None;
    let mut prev_pixel: Option<camera::Pixel> = None;

    while let Some(frame) = source.next_frame() {
        let pixel = detector.detect(&frame);

        let masked_img = detector
            .mask
            .apply_bgr(&frame.image)
            .context("floor-mask apply")?;
        let masked_frame = Frame {
            camera_id: frame.camera_id,
            image: masked_img
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?,
            timestamp: frame.timestamp,
        };

        // 본선이 이번 프레임에 쓴 영역과 동일하게 2·3 스텝을 돌린다.
        let step_roi = if detector.roi.used_roi {
            detector.roi.last_roi
        } else {
            None
        };
        let (step_px, mut cm_panel, mut ct_panel) =
            appearance_steps(&mut appearance, &scorer, &masked_frame, step_roi)?;

        let mut raw = frame
            .image
            .try_clone()
            .map_err(|e| anyhow::anyhow!("raw clone: {e}"))?;
        let mut mask_panel = masked_img;
        detector
            .mask
            .draw_edge_lines(&mut mask_panel, Scalar::new(255.0, 255.0, 0.0, 0.0), 2)?;

        if let Some(r) = detector.roi.last_roi {
            let cyan = Scalar::new(255.0, 255.0, 0.0, 0.0);
            imgproc::rectangle(&mut raw, r, cyan, 2, imgproc::LINE_8, 0)?;
            imgproc::rectangle(&mut mask_panel, r, cyan, 2, imgproc::LINE_8, 0)?;
            imgproc::rectangle(&mut cm_panel, r, cyan, 1, imgproc::LINE_8, 0)?;
            imgproc::rectangle(&mut ct_panel, r, cyan, 1, imgproc::LINE_8, 0)?;
        }

        if let Some(p) = pixel {
            hits += 1;
            let mode = if detector.roi.used_roi { "roi" } else { "full" };
            println!(
                "frame={n} {mode} half={} pixel=({:.1}, {:.1})",
                detector.roi.half_px, p.x, p.y
            );
            Preview::draw_circle_px(&mut raw, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            Preview::draw_circle_px(&mut mask_panel, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            if let Some(prev) = prev_pixel {
                Preview::draw_circle_px(&mut raw, prev, 6, Scalar::new(0.0, 200.0, 255.0, 0.0), 1)?;
                Preview::draw_circle_px(
                    &mut mask_panel,
                    prev,
                    6,
                    Scalar::new(0.0, 200.0, 255.0, 0.0),
                    1,
                )?;
            }
            prev_pixel = last_pixel;
            last_pixel = Some(p);
        } else {
            println!("frame={n} miss");
        }

        if let Some(p) = step_px.or(pixel) {
            Preview::draw_circle_px(&mut cm_panel, p, 8, Scalar::new(0.0, 255.0, 0.0, 0.0), 1)?;
            Preview::draw_circle_px(&mut ct_panel, p, 8, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
        }

        let mut roi_panel = empty_like(&masked_frame)?;
        if let Some(r) = detector.roi.last_roi {
            if let Ok(view) = Mat::roi(&masked_frame.image, r) {
                if let Ok(owned) = view.try_clone() {
                    paste_at(&mut roi_panel, &owned, r)?;
                }
            }
            imgproc::rectangle(
                &mut roi_panel,
                r,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                2,
                imgproc::LINE_8,
                0,
            )?;
            if let Some(p) = pixel {
                Preview::draw_circle_px(
                    &mut roi_panel,
                    p,
                    10,
                    Scalar::new(0.0, 255.0, 0.0, 0.0),
                    2,
                )?;
            }
        } else if let Some(p) = pixel {
            mask_panel.copy_to(&mut roi_panel)?;
            Preview::draw_circle_px(&mut roi_panel, p, 10, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
        }

        let roi_label = if detector.roi.used_roi {
            "4 roi"
        } else if detector.roi.roi_enabled {
            "4 acquire"
        } else {
            "4 roi-off"
        };

        let hit_rate = if n == 0 {
            0.0
        } else {
            100.0 * hits as f64 / (n + 1) as f64
        };
        let equivalent_radius = detector
            .last_area()
            .map(|a| (a / std::f64::consts::PI).sqrt())
            .unwrap_or(0.0);
        let mode = if detector.roi.used_roi {
            "roi"
        } else if detector.roi.roi_enabled {
            "acquire"
        } else {
            "full"
        };
        let appearance_pixel = step_px.or(pixel);
        let colormask_nonzero = nonzero_bgr(&cm_panel);
        let contour_nonzero = nonzero_bgr(&ct_panel);
        let keep_nonzero = opencv::core::count_non_zero(&detector.mask.keep).unwrap_or(0);
        let total_pixels = detector
            .mask
            .width
            .saturating_mul(detector.mask.height)
            .max(1);
        let cut_percent = 100.0 * f64::from(total_pixels - keep_nonzero) / f64::from(total_pixels);

        // BGR: white / cyan / green / orange / yellow
        let white = Scalar::new(255.0, 255.0, 255.0, 0.0);
        let cyan = Scalar::new(255.0, 255.0, 0.0, 0.0);
        let green = Scalar::new(0.0, 255.0, 0.0, 0.0);
        let orange = Scalar::new(0.0, 140.0, 255.0, 0.0);
        let yellow = Scalar::new(0.0, 255.0, 255.0, 0.0);

        // 패널별 HUD (좌상단). 키 안내는 raw만.
        draw_panel_hud(
            &mut raw,
            &[
                format!("cam{}  {detector}", cam_id.0),
                pixel_hud_line(mode, pixel, equivalent_radius),
                format!("hits={hits}/{}  ({hit_rate:.0}%)", n + 1),
            ],
            white,
        )?;
        Preview::draw_help_lines(
            &mut raw,
            &["r ROI | [/] radius_scale | ,/. motion_scale | -/= padding | p paste | q quit"],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;

        let mut mask_hud = vec![format!(
            "floor-edge keep  poly={}",
            detector.mask.keep_poly_len
        )];
        mask_hud.extend(detector.mask.edges.iter().map(|e| {
            format!(
                "{:?} cut={:.3}m δ={:.3}m ({:.0},{:.0})->({:.0},{:.0})",
                e.axis, e.cut, e.margin_m, e.p0.0, e.p0.1, e.p1.0, e.p1.1
            )
        }));
        mask_hud.push(format!(
            "cut={cut_percent:.0}%  keep={keep_nonzero}/{total_pixels}"
        ));
        draw_panel_hud(&mut mask_panel, &mask_hud, cyan)?;

        draw_panel_hud(
            &mut cm_panel,
            &[
                "color gate".to_string(),
                format!("nonzero={colormask_nonzero}"),
                pixel_hud_line("appearance", appearance_pixel, equivalent_radius),
            ],
            green,
        )?;

        draw_panel_hud(
            &mut ct_panel,
            &[
                "color ^ edges".to_string(),
                format!(
                    "nonzero={contour_nonzero}  area=[{:.0},{:.0}]",
                    scorer_params.min_area_px, scorer_params.max_area_px
                ),
                format!("circularity>={:.2}", scorer_params.min_circularity),
                pixel_hud_line("contour pick", appearance_pixel, equivalent_radius),
            ],
            orange,
        )?;

        let roi_box_hud = match detector.roi.last_roi {
            Some(r) => format!("box={}x{} @({},{})", r.width, r.height, r.x, r.y),
            None => "box=full-frame".to_string(),
        };
        draw_panel_hud(
            &mut roi_panel,
            &[
                format!("{mode}  half={}", detector.roi.half_px),
                format!(
                    "radius_scale={:.1}  motion_scale={:.1}  padding={}",
                    detector.roi.params.radius_scale,
                    detector.roi.params.motion_scale,
                    detector.roi.params.padding
                ),
                roi_box_hud,
                pixel_hud_line("detection", pixel, equivalent_radius),
            ],
            yellow,
        )?;

        Preview::draw_cam_label(&mut raw, "0 raw", white)?;
        Preview::draw_cam_label(&mut mask_panel, "1 floor-mask", cyan)?;
        Preview::draw_cam_label(&mut cm_panel, "2 colormask", green)?;
        Preview::draw_cam_label(&mut ct_panel, "3 +contour", orange)?;
        Preview::draw_cam_label(&mut roi_panel, roi_label, yellow)?;

        // 읽는 순서 = 파이프라인: 0→1→2 / 3→4
        let top = Preview::hstack_bgr(&[raw, mask_panel, cm_panel])?;
        let bottom = Preview::hstack_bgr(&[ct_panel, roi_panel])?;
        let mosaic = vstack_bgr(&top, &bottom)?;

        if let Some(dir) = &args.output {
            let out = dir.join(format!("full_{n:04}.png"));
            imgcodecs::imwrite(
                out.to_str().context("out path")?,
                &mosaic,
                &opencv::core::Vector::new(),
            )?;
        }

        if preview {
            match Preview::show_bgr(window, &mosaic, wait_ms)?.action {
                PreviewAction::Quit => break,
                PreviewAction::Key(key) if key == i32::from(b'r') || key == i32::from(b'R') => {
                    detector.set_roi_enabled(!detector.roi.roi_enabled);
                    println!(
                        "roi → {}",
                        if detector.roi.roi_enabled {
                            "on"
                        } else {
                            "off"
                        }
                    );
                }
                PreviewAction::Key(key) => {
                    handle_tune_key(&mut detector.roi, key);
                }
                PreviewAction::Continue => {}
            }
        }

        n += 1;
        if args.images.is_none() && n >= args.max_frames {
            break;
        }
    }

    if preview {
        Preview::destroy_window(window);
    }
    println!("done frames={n} hits={hits} {detector}");
    println!(
        "// paste into RoiParams::default()\n{}",
        detector.roi.params.to_defaults_snippet()
    );
    return Ok(());
}
