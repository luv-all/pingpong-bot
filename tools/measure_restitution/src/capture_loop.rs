//! 멀티캠 캡처 루프 — 반발 e (이 툴 전용 보일러플레이트).

use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{Context, Result, bail};
use opencv::core::Scalar;
use opencv::prelude::*;
use pingpong_bot::{
    BallDetector, BounceEvent, Calibration, CameraId, ColormaskConfig, DetectorKind, FrameSource,
    OpenCvCapture, PixelPoint, Point3, PreviewAction, TrajPoint, build_detector, destroy_window,
    detect_bounces, draw_cam_label, draw_circle_px, draw_debug_lines, draw_world_velocity,
    hstack_bgr, mean_bounce_e, show_bgr, triangulate_views,
};

pub struct CaptureResult {
    pub traj: Vec<TrajPoint>,
    pub bounces: Vec<BounceEvent>,
    pub e: Option<f64>,
}

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

fn open_sources(
    videos: &[PathBuf],
    devices: &[i32],
    fps_override: Option<f64>,
) -> Result<Vec<Box<dyn FrameSource>>> {
    let mut sources = Vec::new();
    if !videos.is_empty() {
        if !devices.is_empty() {
            bail!("--video 와 --device 를 같이 쓰지 마세요");
        }
        for (i, path) in videos.iter().enumerate() {
            let mut cap = OpenCvCapture::from_path(CameraId(i as u8), path)
                .map_err(anyhow::Error::msg)
                .with_context(|| format!("video {}", path.display()))?;
            if let Some(fps) = fps_override {
                cap.set_timeline_fps(fps);
            }
            sources.push(Box::new(cap) as Box<dyn FrameSource>);
        }
        return Ok(sources);
    }
    if !devices.is_empty() {
        for (i, &dev) in devices.iter().enumerate() {
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

/// OpenCV: open → read → detect/triangulate/draw → q 종료.
pub fn run_capture(
    calibration: &Path,
    videos: &[PathBuf],
    devices: &[i32],
    detector: DetectorKind,
    preview: bool,
    wait_ms: i32,
    max_frames: usize,
    fps_override: Option<f64>,
) -> Result<CaptureResult> {
    let calibration = load_calibration(calibration)?;
    let mut sources = open_sources(videos, devices, fps_override)?;
    if sources.len() < 2 {
        bail!("카메라 소스 ≥2 필요");
    }
    if sources.len() > calibration.camera_count() {
        bail!(
            "소스 {}대 > calibration {}대",
            sources.len(),
            calibration.camera_count()
        );
    }

    let mut detectors: Vec<Box<dyn BallDetector>> = (0..sources.len())
        .map(|_| build_detector(detector, ColormaskConfig::default()))
        .collect();

    let window = "measure:restitution";
    let mut traj = Vec::new();
    let mut n = 0usize;
    let mut epoch: Option<Instant> = None;

    loop {
        if n >= max_frames {
            break;
        }

        let mut panels = Vec::with_capacity(sources.len());
        let mut hits = Vec::new();
        let mut frame0_ts = None;
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
        let e_mean = mean_bounce_e(&bounces);

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

        let mut lines = vec![format!(
            "restitution  frame={n}  hits={}/{}  traj={}",
            hits.len(),
            sources.len(),
            traj.len()
        )];
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
        } else {
            lines.push("bounce: waiting".into());
        }
        if let Some(e) = e_mean {
            lines.push(format!("mean e={e:.4}  (n={})", bounces.len()));
        }
        lines.push("q/ESC quit".into());

        let mut mosaic = hstack_bgr(&panels)?;
        draw_debug_lines(&mut mosaic, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;

        if preview {
            match show_bgr(window, &mosaic, wait_ms)? {
                PreviewAction::Quit => break,
                PreviewAction::Continue | PreviewAction::Key(_) => {}
            }
        }
        n += 1;
    }

    if preview {
        destroy_window(window);
    }

    let bounces = detect_bounces(&traj);
    return Ok(CaptureResult {
        e: mean_bounce_e(&bounces),
        traj,
        bounces,
    });
}
