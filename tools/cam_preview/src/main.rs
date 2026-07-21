//! 다중 웹캠 프리뷰 — `DEVICES`를 가로로 이어 붙인 한 창.
//!
//! 장치 인덱스는 아래 `DEVICES`만 수정.
//! - `q` / ESC 종료
//! - `Space` 동결/해제
//! - `e` 짧은 노출 시도 (macOS OpenCV/AVFoundation에선 대개 무시됨)
//!
//! 모자이크는 `pingpong_bot::hstack_bgr` (최대 높이 + 패딩, 손실 없음).

use std::time::Instant;

use anyhow::{Context, Result, bail};
use opencv::core::Scalar;
use opencv::prelude::*;
use pingpong_bot::{
    CameraId, FrameSource, OpenCvCapture, PreviewAction, destroy_window, draw_cam_label,
    draw_debug_lines, draw_help_lines, hstack_bgr, show_bgr,
};

/// 웹캠 인덱스들 (여기만 수정).
/// 맥북 기준 보통 0: global shutter camera 외장으로 단 것, 1: 아이폰
const DEVICES: &[i32] = &[0, 1];

/// Arducam B0332(OV9281): MJPG@1280x800 → 최대 120fps. YUY2면 ~10fps로 떨어짐.
const STREAM_W: i32 = 1280;
const STREAM_H: i32 = 800;
const STREAM_FPS: f64 = 120.0;
const STREAM_FOURCC: &[u8; 4] = b"MJPG";

struct FpsMeter {
    last: Option<Instant>,
    fps: f64,
}

impl FpsMeter {
    fn new() -> Self {
        return Self {
            last: None,
            fps: 0.0,
        };
    }

    fn tick(&mut self) {
        let now = Instant::now();
        if let Some(prev) = self.last {
            let dt = now.duration_since(prev).as_secs_f64();
            if dt > 1e-4 {
                let instant = 1.0 / dt;
                self.fps = if self.fps <= 0.0 {
                    instant
                } else {
                    self.fps * 0.85 + instant * 0.15
                };
            }
        }
        self.last = Some(now);
    }
}

struct CamSlot {
    device: i32,
    cap: OpenCvCapture,
    meter: FpsMeter,
    /// 마지막 표시용 패널 (동결 시 유지).
    panel: Option<Mat>,
}

fn main() -> Result<()> {
    if DEVICES.is_empty() {
        bail!("DEVICES 가 비어 있음");
    }

    let mut cams: Vec<CamSlot> = Vec::with_capacity(DEVICES.len());
    let mut exp_supported = true;
    for (i, &id) in DEVICES.iter().enumerate() {
        let mut cap = OpenCvCapture::from_device(CameraId(i as u8), id)
            .map_err(anyhow::Error::msg)
            .with_context(|| format!("device {id}"))?;
        cap.request_stream(STREAM_W, STREAM_H, STREAM_FPS, STREAM_FOURCC)
            .map_err(anyhow::Error::msg)
            .with_context(|| format!("device {id}: stream request"))?;
        let ro = cap.exposure_readout();
        if ro.likely_unsupported() {
            exp_supported = false;
        }
        println!(
            "device {id}: backend={} fourcc={} fps={:?} size={:?} | {}",
            ro.backend,
            cap.reported_fourcc().unwrap_or_else(|| "?".into()),
            cap.reported_fps(),
            cap.reported_size(),
            ro.summary_line()
        );
        cams.push(CamSlot {
            device: id,
            cap,
            meter: FpsMeter::new(),
            panel: None,
        });
    }
    if !exp_supported {
        println!(
            "note: OpenCV macOS(AVFoundation) ignores UVC exposure — `e` will not change the image"
        );
    }

    let window = "cam_preview";
    let mut frozen = false;
    let mut short_exposure = false;
    println!("devices={DEVICES:?}  Space=freeze  e=short exposure  q/ESC=quit");

    loop {
        for cam in &mut cams {
            let Some(frame) = cam.cap.next_frame() else {
                bail!("device {}: 프레임 끝/실패", cam.device);
            };
            cam.meter.tick();

            // 동결 중에도 grab은 해서 USB 버퍼가 쌓이지 않게 한다.
            if frozen {
                continue;
            }

            let mut panel = frame
                .image
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            let (w, h) = (panel.cols(), panel.rows());
            let ro = cam.cap.exposure_readout();

            let mut lines = vec![format!("cam{}", cam.device), format!("{w}x{h}")];
            let fourcc = cam.cap.reported_fourcc().unwrap_or_else(|| "?".into());
            match cam.cap.reported_fps() {
                Some(cap_fps) => lines.push(format!(
                    "fps {:.1} meas / {:.0} cap  {fourcc}",
                    cam.meter.fps, cap_fps
                )),
                None => lines.push(format!("fps {:.1} meas  {fourcc}", cam.meter.fps)),
            }
            lines.push(ro.summary_line());
            if short_exposure {
                if ro.likely_unsupported() {
                    lines.push("exp short (ignored)".into());
                } else {
                    lines.push("exp short".into());
                }
            }
            if let Some((rw, rh)) = cam.cap.reported_size() {
                if rw != w || rh != h {
                    lines.push(format!("cap {rw}x{rh}"));
                }
            }

            draw_debug_lines(&mut panel, &lines, Scalar::new(0.0, 255.0, 255.0, 0.0))?;
            draw_cam_label(
                &mut panel,
                &format!("cam{}", cam.device),
                Scalar::new(0.0, 255.0, 255.0, 0.0),
            )?;
            cam.panel = Some(panel);
        }

        let mut panels = Vec::with_capacity(cams.len());
        for cam in &cams {
            let Some(panel) = &cam.panel else {
                bail!("device {}: 첫 프레임 없음", cam.device);
            };
            let mut shown = panel
                .try_clone()
                .map_err(|e| anyhow::anyhow!("clone: {e}"))?;
            if frozen {
                draw_cam_label(&mut shown, "FROZEN", Scalar::new(0.0, 0.0, 255.0, 0.0))?;
            }
            panels.push(shown);
        }

        let mut mosaic = hstack_bgr(&panels)?;
        let help_exp = if !exp_supported {
            "e N/A(mac)"
        } else if short_exposure {
            "e short"
        } else {
            "e auto"
        };
        let help = ["Space freeze", help_exp, "q/ESC quit"];
        draw_help_lines(&mut mosaic, &help, Scalar::new(0.0, 255.0, 80.0, 0.0))?;
        match show_bgr(window, &mosaic, 1)? {
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
                    let ok = if short_exposure {
                        cam.cap.request_short_exposure()
                    } else {
                        cam.cap.request_auto_exposure()
                    };
                    any_ok |= ok;
                    let ro = cam.cap.exposure_readout();
                    println!(
                        "device {}: set_ok={ok} backend={} | {}",
                        cam.device,
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

    destroy_window(window);
    return Ok(());
}
