//! 멀티캠 캡처 루프 — 반발 e (이 툴 전용 보일러플레이트).

use std::path::Path;
use std::time::Instant;

use anyhow::{Result, bail};
use opencv::core::Scalar;
use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::camera::{Calibration, FrameSource, Preview, PreviewAction, StereoOfflineArgs};
use pingpong_bot::defaults::detector_for;
use pingpong_bot::detector::Detector;
use pingpong_bot::estimator;

pub struct CaptureResult {
    pub traj: Vec<estimator::TrajPoint>,
    pub bounces: Vec<estimator::BounceEvent>,
    pub e: Option<f64>,
}

fn open_sources(
    cam: &camera::CamCliArgs,
    offline: &StereoOfflineArgs,
    timeline_fps: Option<f64>,
) -> Result<Vec<Box<dyn FrameSource>>> {
    let (sources, _) = cam
        .open_stereo_input(offline, timeline_fps)
        .map_err(anyhow::Error::msg)?;
    return Ok(sources);
}

/// OpenCV: open → read → detect/triangulate/draw → q 종료.
pub fn run_capture(
    calibration: &Path,
    cam: &camera::CamCliArgs,
    offline: &StereoOfflineArgs,
    preview: bool,
    wait_ms: i32,
    max_frames: usize,
    timeline_fps: Option<f64>,
) -> Result<CaptureResult> {
    let calibration = Calibration::load_json(calibration).map_err(anyhow::Error::msg)?;
    if calibration.camera_count() < 2 {
        bail!("카메라 ≥2 필요 (got {})", calibration.camera_count());
    }
    let mut sources = open_sources(cam, offline, timeline_fps)?;
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

    let ids: Vec<_> = sources.iter().map(|s| s.camera_id()).collect();
    let mut detectors: Vec<Detector> = ids
        .iter()
        .map(|&id| detector_for(id))
        .collect::<Result<Vec<_>>>()?;

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
            let cam_id = camera::Id(i as u8);
            let pixel = detectors[i].detect(&frame);
            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            if let Some(p) = pixel {
                hits.push((cam_id, p));
                Preview::draw_circle_px(&mut panel, p, 8, Scalar::new(0.0, 255.0, 0.0, 0.0), 2)?;
            }
            Preview::draw_cam_label(
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
        if let Some(pos) = estimator::Triangulate::pixels(&hits, &calibration) {
            traj.push(estimator::TrajPoint {
                t: sync_t,
                pos,
                pixels: hits.clone(),
            });
        }

        let bounces = estimator::TrajAnalysis::detect_bounces(&traj);
        let e_mean = estimator::TrajAnalysis::mean_bounce_e(&bounces);

        if let Some(ev) = bounces.last() {
            for (i, panel) in panels.iter_mut().enumerate() {
                let Some(params) = calibration.params(camera::Id(i as u8)) else {
                    continue;
                };
                if let Some(px) = params.project_world(ev.contact) {
                    Preview::draw_circle_px(panel, px, 12, Scalar::new(255.0, 0.0, 255.0, 0.0), 2)?;
                }
                if let Some(px) = params.project_world(ev.prev) {
                    Preview::draw_circle_px(panel, px, 7, Scalar::new(0.0, 220.0, 255.0, 0.0), 2)?;
                }
                if let Some(px) = params.project_world(ev.next) {
                    Preview::draw_circle_px(panel, px, 7, Scalar::new(0.0, 128.0, 255.0, 0.0), 2)?;
                }
                Preview::draw_world_velocity(
                    panel,
                    params,
                    ev.contact,
                    ev.v_in,
                    0.08,
                    Scalar::new(0.0, 0.0, 255.0, 0.0),
                )?;
                Preview::draw_world_velocity(
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
                ev.contact.coords.x, ev.contact.coords.y, ev.contact.coords.z
            ));
        } else {
            lines.push("bounce: waiting".into());
        }
        if let Some(e) = e_mean {
            lines.push(format!("mean e={e:.4}  (n={})", bounces.len()));
        }

        let mut mosaic = Preview::hstack_bgr(&panels)?;
        Preview::draw_debug_lines(&mut mosaic, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
        Preview::draw_help_lines(
            &mut mosaic,
            &["q/ESC quit"],
            Scalar::new(0.0, 255.0, 80.0, 0.0),
        )?;

        if preview {
            match Preview::show_bgr(window, &mosaic, wait_ms)?.action {
                PreviewAction::Quit => break,
                PreviewAction::Continue | PreviewAction::Key(_) => {}
            }
        }
        n += 1;
    }

    if preview {
        Preview::destroy_window(window);
    }

    let bounces = estimator::TrajAnalysis::detect_bounces(&traj);
    return Ok(CaptureResult {
        e: estimator::TrajAnalysis::mean_bounce_e(&bounces),
        traj,
        bounces,
    });
}
