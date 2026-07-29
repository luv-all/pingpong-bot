//! OpenCV `VideoCapture` 장치 인덱스 프로브.
//!
//! USB를 다시 꽂으면 인덱스가 바뀐다. OBS에 보여도 OpenCV 번호는 다를 수 있으니
//! 이 툴로 `LEFT_DEVICE` / `RIGHT_DEVICE` (`defaults::calib`)를 맞춘다.
//!
//! hinguri 시절 `cv2_enumerate_cameras`와 같은 역할 (이름·VID는 OS API 없이
//! 인덱스·백엔드·프레임 성공 여부만 본다).

use anyhow::{Result, bail};
use clap::Parser;
use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::camera::{
    CamRigConfig, CaptureBackend, FrameSource, OpenCvCapture, Preview, PreviewAction,
};

#[derive(Parser, Debug)]
#[command(name = "cam-list")]
#[command(about = "Probe OpenCV camera device indices for the chosen backend")]
struct Args {
    /// OpenCV 백엔드: recommended|msmf|dshow|any|v4l2|avfoundation
    #[arg(long, default_value = "recommended")]
    backend: String,

    /// msmf + dshow (+ any)를 각각 프로브
    #[arg(long)]
    all_backends: bool,

    /// 시도할 최대 인덱스 (0..max_index inclusive 아님, 0..max_index)
    #[arg(long, default_value_t = 8)]
    max_index: i32,

    /// 열린 장치마다 한 프레임 프리뷰 (q로 다음)
    #[arg(long)]
    preview: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.max_index <= 0 {
        bail!("--max-index 는 1 이상");
    }

    let rig = CamRigConfig::default();
    println!(
        "defaults::calib 현재 매핑: LEFT_DEVICE={}  RIGHT_DEVICE={}",
        rig.left_device, rig.right_device
    );
    println!("(보이면 해당 숫자를 calib.rs에 넣으면 됨)\n");

    let backends = if args.all_backends {
        backends_for_host()
    } else {
        vec![CaptureBackend::parse(&args.backend).map_err(anyhow::Error::msg)?]
    };

    for backend in backends {
        probe_backend(backend, args.max_index, args.preview)?;
        println!();
    }

    println!("다음: src/defaults/calib.rs 의 LEFT_DEVICE / RIGHT_DEVICE 수정 후");
    println!("  cargo run -p cam-preview -- --cam left");
    println!("  cargo run -p cam-preview -- --cam right");
    return Ok(());
}

fn backends_for_host() -> Vec<CaptureBackend> {
    let mut v = vec![CaptureBackend::recommended()];
    if cfg!(target_os = "windows") {
        for b in [
            CaptureBackend::Msmf,
            CaptureBackend::DShow,
            CaptureBackend::Any,
        ] {
            if !v.contains(&b) {
                v.push(b);
            }
        }
    } else {
        if !v.contains(&CaptureBackend::Any) {
            v.push(CaptureBackend::Any);
        }
    }
    return v;
}

fn probe_backend(backend: CaptureBackend, max_index: i32, preview: bool) -> Result<()> {
    println!(
        "=== backend={} (api={}) ===",
        backend.as_str(),
        backend.api_pref()
    );
    let mut found = 0usize;

    for index in 0..max_index {
        // Id는 논리 id — 프로브에선 인덱스와 같게 둠 (0..255)
        let id = camera::Id(index.clamp(0, 255) as u8);
        match OpenCvCapture::from_device_with_backend(id, index, backend) {
            Ok(mut cap) => {
                let summary = cap.stream_summary();
                let frame = cap.next_frame();
                let frame_ok = frame.is_some();
                let size = frame
                    .as_ref()
                    .map(|f| format!("{}x{}", f.image.cols(), f.image.rows()))
                    .unwrap_or_else(|| "-".into());
                println!(
                    "  device {index}: OPEN  frame={} grabbed={size} | {summary}",
                    if frame_ok { "ok" } else { "FAIL" }
                );
                found += 1;

                if preview {
                    if let Some(f) = frame {
                        let window = format!("cam_list device {index} ({})", backend.as_str());
                        println!("    preview: q/ESC → next device");
                        loop {
                            match Preview::show_bgr(&window, &f.image, 30)?.action {
                                PreviewAction::Quit => break,
                                PreviewAction::Continue | PreviewAction::Key(_) => {}
                            }
                        }
                        Preview::destroy_window(&window);
                    } else {
                        println!("    preview skipped (no frame)");
                    }
                }
            }
            Err(e) => {
                let short = e.lines().next().unwrap_or(e.as_str());
                println!("  device {index}: —  ({short})");
            }
        }
    }

    if found == 0 {
        println!("  (열린 장치 없음 — 권한/점유/백엔드 확인. OBS가 켜져 있으면 닫고 재시도)");
    } else {
        println!("  → {found} device(s) opened");
    }
    return Ok(());
}
