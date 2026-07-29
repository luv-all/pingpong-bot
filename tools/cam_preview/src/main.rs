//! 다중 웹캠 프리뷰 — `--cam` 역할을 가로로 이어 붙인 한 창.
//!
//! - `q` / ESC 종료
//! - `Space` 동결/해제
//! - `e` 짧은 노출 시도 (macOS OpenCV/AVFoundation에선 대개 무시됨)
//!
//! 모자이크는 `Preview::hstack_bgr` (최대 높이 + 패딩, 손실 없음).
//! 표시만 모니터보다 클 때 downscale (`Preview::show_bgr`).

mod args;
mod cam_slot;
mod fps_meter;
mod live_source;

use anyhow::{Result, bail};
use clap::Parser;
use opencv::core::Scalar;
use opencv::prelude::*;
use pingpong_bot::camera::{OpenCvCapture, Preview, PreviewAction, ThreadedCapture};

use args::Args;
use cam_slot::CamSlot;
use fps_meter::FpsMeter;
use live_source::LiveSource;

fn main() -> Result<()> {
    let args = Args::parse();
    let cam = args.cam.as_cam_cli();
    let backend = cam.stream.backend().map_err(anyhow::Error::msg)?;
    let resolved = cam.resolve().map_err(anyhow::Error::msg)?;
    if resolved.is_empty() {
        bail!("--cam 이 비어 있음");
    }

    let mut cams: Vec<CamSlot> = Vec::with_capacity(resolved.len());
    let mut exp_supported = true;
    for r in resolved {
        let mut cap = OpenCvCapture::from_device_with_backend(r.camera_id, r.device, backend)
            .map_err(anyhow::Error::msg)?;
        cam.stream.apply(&mut cap).map_err(anyhow::Error::msg)?;

        let ro = cap.exposure_readout();
        if ro.likely_unsupported() {
            exp_supported = false;
        }
        let fourcc_label = cap.reported_fourcc().unwrap_or_else(|| "?".into());
        let reported_fps = cap.reported_fps();
        let reported_size = cap.reported_size();
        let exposure_backend = ro.backend.clone();
        let label = format!("{}#{} (device {})", r.role, r.camera_id.0, r.device);

        let source = if cam.stream.threaded {
            LiveSource::Threaded(ThreadedCapture::spawn(cap))
        } else {
            LiveSource::Direct(cap)
        };
        cams.push(CamSlot {
            label,
            source,
            fourcc_label,
            reported_fps,
            reported_size,
            exposure_backend,
            meter: FpsMeter::new(),
            panel: None,
        });
    }
    if !exp_supported {
        println!(
            "note: OpenCV macOS(AVFoundation) ignores UVC exposure — `e` will not change the image"
        );
    }
    if cam.stream.threaded {
        println!("threaded grab: on (meas=display fps, grab=capture thread fps)");
    }

    let window = "cam_preview";
    let mut frozen = false;
    let mut short_exposure = false;
    let s = &cam.stream;
    let (req_w, req_h) = s.resolved_size();
    println!(
        "cams={}  request={}x{}@{:.0} {} backend={} threaded={}  Space=freeze  e=short exposure  q/ESC=quit",
        cams.iter()
            .map(|c| c.label.as_str())
            .collect::<Vec<_>>()
            .join(","),
        req_w,
        req_h,
        s.fps,
        s.fourcc,
        s.backend,
        s.threaded
    );

    loop {
        for cam in &mut cams {
            let Some(frame) = cam.source.next_frame() else {
                bail!(
                    "{}: 프레임 없음 — USB device 매핑(defaults::calib LEFT/RIGHT_DEVICE) 또는 백엔드/스레드 확인. OBS에 보이면 연결 문제는 아님",
                    cam.label
                );
            };
            cam.meter.tick();

            if frozen {
                continue;
            }

            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            let (w, h) = (panel.cols(), panel.rows());

            let mut lines = vec![cam.label.clone(), format!("{w}x{h}")];
            match cam.reported_fps {
                Some(cap_fps) => lines.push(format!(
                    "fps {:.1} meas / {:.0} cap  {}",
                    cam.meter.fps, cap_fps, cam.fourcc_label
                )),
                None => lines.push(format!(
                    "fps {:.1} meas  {}",
                    cam.meter.fps, cam.fourcc_label
                )),
            }
            if let Some(grab) = cam.source.capture_fps() {
                if grab > 0.0 {
                    lines.push(format!("grab {grab:.1}"));
                }
            }
            lines.push(format!("be {}", cam.exposure_backend));
            if short_exposure {
                lines.push("exp short".into());
            }
            if let Some((rw, rh)) = cam.reported_size {
                if rw != w || rh != h {
                    lines.push(format!("cap {rw}x{rh}"));
                }
            }

            Preview::draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            Preview::draw_cam_label(&mut panel, &cam.label, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            cam.panel = Some(panel);
        }

        let mut panels = Vec::with_capacity(cams.len());
        for cam in &cams {
            let Some(panel) = &cam.panel else {
                bail!("{}: 첫 프레임 없음", cam.label);
            };
            let mut shown = panel
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            if frozen {
                Preview::draw_cam_label(&mut shown, "FROZEN", Scalar::new(0.0, 0.0, 255.0, 0.0))?;
            }
            panels.push(shown);
        }

        let mut mosaic = Preview::hstack_bgr(&panels)?;
        let help_exp = if !exp_supported {
            "e N/A(mac)"
        } else if short_exposure {
            "e short"
        } else {
            "e auto"
        };
        let help = ["Space freeze", help_exp, "q/ESC quit"];
        Preview::draw_help_lines(&mut mosaic, &help, Scalar::new(0.0, 255.0, 80.0, 0.0))?;
        match Preview::show_bgr(window, &mosaic, 1)?.action {
            PreviewAction::Quit => break,
            PreviewAction::Continue => {}
            PreviewAction::Key(key) if key == i32::from(b' ') => {
                frozen = !frozen;
                println!("{}", if frozen { "frozen" } else { "live" });
            }
            PreviewAction::Key(key) if key == i32::from(b'e') || key == i32::from(b'E') => {
                short_exposure = !short_exposure;
                let mut any_ok = false;
                for cam in &mut cams {
                    let Some(cap) = cam.source.as_capture_mut() else {
                        println!(
                            "{}: exposure N/A in --threaded (restart without it to tune)",
                            cam.label
                        );
                        continue;
                    };
                    let ok = if short_exposure {
                        cap.request_short_exposure()
                    } else {
                        cap.request_auto_exposure()
                    };
                    any_ok |= ok;
                    let ro = cap.exposure_readout();
                    println!(
                        "{}: set_ok={ok} backend={} | {}",
                        cam.label,
                        ro.backend,
                        ro.summary_line()
                    );
                }
                if !any_ok {
                    println!(
                        "exposure unchanged — OpenCV on this OS cannot drive UVC exposure (use bright light / Linux|Windows, or a native UVC tool)"
                    );
                }
            }
            PreviewAction::Key(_) => {}
        }
    }

    Preview::destroy_window(window);
    let _ = cams;
    return Ok(());
}
