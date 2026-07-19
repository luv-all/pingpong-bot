//! 멀티캠 영상/장치 → 검출 → 삼각측량 → 궤적 (measure_* 공용).

use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{Context, Result, bail};
use nalgebra::Vector3;
use opencv::core::Scalar;
use opencv::prelude::*;

use crate::camera::preview::{
    PreviewAction, destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines,
    draw_world_velocity, hstack_bgr, show_bgr,
};
use crate::estimator::traj_measure::{
    BounceEvent, RollEvent, TrajPoint, detect_bounces, detect_rolls, mean_bounce_e, mean_roll_mu,
};
use crate::triangulate_views;
use crate::{
    BallDetector, Calibration, CameraId, ColormaskConfig, DetectorKind, FrameSource, OpenCvCapture,
    PixelPoint, Point3, build_detector,
};

/// 측정 모드.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasureKind {
    Restitution,
    Friction,
}

#[derive(Debug, Clone)]
pub struct MeasureVideoOptions {
    pub calibration: PathBuf,
    pub videos: Vec<PathBuf>,
    pub devices: Vec<i32>,
    pub detector: DetectorKind,
    pub preview: bool,
    pub wait_ms: i32,
    pub max_frames: usize,
    /// 파일 FPS 덮어쓰기 (타임스탬프용). None이면 캡처 속성.
    pub fps_override: Option<f64>,
}

pub struct MeasureVideoResult {
    pub traj: Vec<TrajPoint>,
    pub bounces: Vec<BounceEvent>,
    pub rolls: Vec<RollEvent>,
    pub e: Option<f64>,
    pub mu: Option<f64>,
}

pub fn load_calibration(path: &Path) -> Result<Calibration> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("calibration 읽기: {}", path.display()))?;
    let cal: Calibration = serde_json::from_str(&text)
        .with_context(|| format!("calibration JSON: {}", path.display()))?;
    if cal.camera_count() < 2 {
        bail!(
            "캘리브레이션에 카메라가 2대 이상 필요 (got {})",
            cal.camera_count()
        );
    }
    return Ok(cal);
}

fn open_sources(opts: &MeasureVideoOptions) -> Result<Vec<Box<dyn FrameSource>>> {
    let mut sources = Vec::new();
    if !opts.videos.is_empty() {
        if !opts.devices.is_empty() {
            bail!("--video 와 --device 를 같이 쓰지 마세요");
        }
        for (i, path) in opts.videos.iter().enumerate() {
            let mut cap = OpenCvCapture::from_path(CameraId(i as u8), path)
                .map_err(anyhow::Error::msg)
                .with_context(|| format!("video {}", path.display()))?;
            if let Some(fps) = opts.fps_override {
                cap.set_timeline_fps(fps);
            }
            sources.push(Box::new(cap) as Box<dyn FrameSource>);
        }
        return Ok(sources);
    }
    if !opts.devices.is_empty() {
        for (i, &dev) in opts.devices.iter().enumerate() {
            let cap = OpenCvCapture::from_device(CameraId(i as u8), dev)
                .map_err(anyhow::Error::msg)
                .with_context(|| format!("device {dev}"))?;
            sources.push(Box::new(cap) as Box<dyn FrameSource>);
        }
        return Ok(sources);
    }
    bail!("--video PATH (반복) 또는 --device N (반복) 필요 — 카메라 ≥2");
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

/// 멀티캠 루프: 프리뷰에 과정(v_in/v_out, 전후 프레임 원)을 그린다.
pub fn run_measure_video(
    kind: MeasureKind,
    opts: &MeasureVideoOptions,
) -> Result<MeasureVideoResult> {
    let calibration = load_calibration(&opts.calibration)?;
    let mut sources = open_sources(opts)?;
    if sources.len() < 2 {
        bail!("카메라 소스 ≥2 필요 (got {})", sources.len());
    }
    if sources.len() > calibration.camera_count() {
        bail!(
            "소스 {}대 > calibration 카메라 {}대",
            sources.len(),
            calibration.camera_count()
        );
    }

    let mut detectors: Vec<Box<dyn BallDetector>> = (0..sources.len())
        .map(|_| build_detector(opts.detector, ColormaskConfig::default()))
        .collect();

    let window = match kind {
        MeasureKind::Restitution => "measure:restitution",
        MeasureKind::Friction => "measure:friction",
    };

    let mut traj: Vec<TrajPoint> = Vec::new();
    let mut n = 0usize;
    let mut epoch: Option<Instant> = None;

    loop {
        if n >= opts.max_frames {
            break;
        }

        let mut panels = Vec::with_capacity(sources.len());
        let mut hits: Vec<(CameraId, PixelPoint)> = Vec::new();
        let mut frame0_ts: Option<Instant> = None;
        let mut any = false;
        let mut all_ok = true;

        for (i, source) in sources.iter_mut().enumerate() {
            let Some(frame) = source.next_frame() else {
                all_ok = false;
                break;
            };
            any = true;
            if i == 0 {
                frame0_ts = Some(frame.timestamp);
            }
            let cam_id = CameraId(i as u8);
            let pixel = detectors[i].detect(&frame);
            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;

            if let Some(p) = pixel {
                hits.push((cam_id, p));
                draw_circle_px(&mut panel, p, 8, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            }
            draw_cam_label(
                &mut panel,
                &format!("cam{i}"),
                Scalar::new(255.0, 255.0, 255.0, 0.0),
            )?;
            panels.push(panel);
        }

        if !all_ok || !any {
            break;
        }

        let ts = frame0_ts.unwrap_or_else(Instant::now);
        let sync_t = match epoch {
            Some(e) => ts.duration_since(e).as_secs_f64(),
            None => {
                epoch = Some(ts);
                0.0
            }
        };

        if let Some(pos) = triangulate_pixels(&hits, &calibration) {
            traj.push(TrajPoint {
                t: sync_t,
                pos,
                pixels: hits.clone(),
            });
        }

        let bounces = detect_bounces(&traj);
        let rolls = detect_rolls(&traj);
        let e_mean = mean_bounce_e(&bounces);
        let mu_mean = mean_roll_mu(&rolls);

        if let Some(ev) = bounces.last() {
            for (i, panel) in panels.iter_mut().enumerate() {
                let Some(params) = calibration.params(CameraId(i as u8)) else {
                    continue;
                };
                if let Some(px) = params.project_world(ev.contact) {
                    draw_circle_px(panel, px, 12, Scalar::new(255.0, 0.0, 255.0, 0.0), 2)?;
                }
                if let Some(px) = params.project_world(ev.prev) {
                    draw_circle_px(panel, px, 7, Scalar::new(0.0, 220.0, 255.0, 0.0), 2)?;
                }
                if let Some(px) = params.project_world(ev.next) {
                    draw_circle_px(panel, px, 7, Scalar::new(0.0, 128.0, 255.0, 0.0), 2)?;
                }
                draw_world_velocity(
                    panel,
                    params,
                    ev.contact,
                    ev.v_in,
                    0.08,
                    Scalar::new(0.0, 0.0, 255.0, 0.0),
                )?;
                draw_world_velocity(
                    panel,
                    params,
                    ev.contact,
                    ev.v_out,
                    0.08,
                    Scalar::new(0.0, 255.0, 0.0, 0.0),
                )?;
            }
        }
        if kind == MeasureKind::Friction {
            if let Some(ev) = rolls.last() {
                for (i, panel) in panels.iter_mut().enumerate() {
                    let Some(params) = calibration.params(CameraId(i as u8)) else {
                        continue;
                    };
                    if let Some(px) = params.project_world(ev.p0) {
                        draw_circle_px(panel, px, 9, Scalar::new(255.0, 200.0, 0.0, 0.0), 2)?;
                    }
                    if let Some(px) = params.project_world(ev.p1) {
                        draw_circle_px(panel, px, 9, Scalar::new(0.0, 255.0, 255.0, 0.0), 2)?;
                    }
                }
            }
        }

        let mut lines: Vec<String> = vec![format!(
            "{}  frame={n}  hits={}/{}  traj={}",
            match kind {
                MeasureKind::Restitution => "restitution",
                MeasureKind::Friction => "friction",
            },
            hits.len(),
            sources.len(),
            traj.len()
        )];
        match kind {
            MeasureKind::Restitution => {
                if let Some(ev) = bounces.last() {
                    lines.push(format!("bounce#{}  e={:.4}", bounces.len(), ev.e));
                    lines.push(format!(
                        "v_in=({:.2},{:.2},{:.2})",
                        ev.v_in.x, ev.v_in.y, ev.v_in.z
                    ));
                    lines.push(format!(
                        "v_out=({:.2},{:.2},{:.2})",
                        ev.v_out.x, ev.v_out.y, ev.v_out.z
                    ));
                    lines.push(format!(
                        "contact=({:.3},{:.3},{:.3})",
                        ev.contact.v.x, ev.contact.v.y, ev.contact.v.z
                    ));
                    lines.push(format!(
                        "prev→next z {:.3}→{:.3}→{:.3}",
                        ev.prev.v.z, ev.contact.v.z, ev.next.v.z
                    ));
                } else {
                    lines.push("bounce: waiting (need vz flip near table)".into());
                }
                if let Some(e) = e_mean {
                    lines.push(format!("mean e={e:.4}  (n={})", bounces.len()));
                }
            }
            MeasureKind::Friction => {
                if let Some(ev) = rolls.last() {
                    lines.push(format!("roll#{}  μ={:.4}", rolls.len(), ev.mu));
                    lines.push(format!("vt_in={:.3}  vt_out={:.3}", ev.vt_in, ev.vt_out));
                    lines.push(format!(
                        "p0=({:.3},{:.3},{:.3})",
                        ev.p0.v.x, ev.p0.v.y, ev.p0.v.z
                    ));
                    lines.push(format!(
                        "p1=({:.3},{:.3},{:.3})",
                        ev.p1.v.x, ev.p1.v.y, ev.p1.v.z
                    ));
                } else {
                    lines.push("roll: waiting (on-table, low |vz|)".into());
                }
                if let Some(mu) = mu_mean {
                    lines.push(format!("mean μ={mu:.4}  (n={})", rolls.len()));
                }
            }
        }
        lines.push("q/ESC quit · green=detect · magenta=contact".into());
        lines.push("cyan=prev · orange=next · red=v_in · lime=v_out".into());

        let mut mosaic = hstack_bgr(&panels)?;
        draw_debug_lines(&mut mosaic, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;

        if opts.preview {
            match show_bgr(window, &mosaic, opts.wait_ms)? {
                PreviewAction::Quit => break,
                PreviewAction::Continue => {}
            }
        }

        if n % 30 == 0 {
            println!(
                "frame={n} traj={} bounces={} rolls={} e={:?} mu={:?}",
                traj.len(),
                bounces.len(),
                rolls.len(),
                e_mean,
                mu_mean
            );
        }

        n += 1;
    }

    if opts.preview {
        destroy_window(window);
    }

    let bounces = detect_bounces(&traj);
    let rolls = detect_rolls(&traj);
    return Ok(MeasureVideoResult {
        e: mean_bounce_e(&bounces),
        mu: mean_roll_mu(&rolls),
        traj,
        bounces,
        rolls,
    });
}

/// 속도 벡터 디버그 한 줄.
pub fn format_vec3(v: Vector3<f64>) -> String {
    return format!("({:.3},{:.3},{:.3})", v.x, v.y, v.z);
}
