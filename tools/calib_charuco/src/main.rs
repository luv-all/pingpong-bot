//! ChArUco 보드 촬영 → 코너 검출 → 카메라 내부 파라미터·왜곡 계산.
//!
//! 산출물: `Calibration` JSON → 런타임 TOML의 `calibration_path`로 로드.
//! 외부 R|t는 피팅하지 않음 (sim 자리표시자). 카메라 1대분 이미지 폴더 기준.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_bot::{Calibration, CameraId, CharucoBoardSpec, calibrate_charuco};

#[derive(Parser)]
#[command(name = "calib_charuco", about = "ChArUco 카메라 보정 도구")]
struct Args {
    /// sim 기본 배치 Calibration JSON을 내보낸다.
    #[arg(long)]
    emit_sim: Option<u8>,

    /// 출력 JSON 경로
    #[arg(short = 'o', long, default_value = "calibration.json")]
    output: PathBuf,

    /// 기존 Calibration JSON 검증(로드만)
    #[arg(long)]
    validate: Option<PathBuf>,

    /// OpenCV ChArUco 실보정 (인트린식 + dist)
    #[arg(long)]
    from_images: Option<PathBuf>,

    /// 출력 CameraId (기본 0)
    #[arg(long, default_value_t = 0)]
    camera_id: u8,

    /// 보드 squares X (기본 5)
    #[arg(long, default_value_t = 5)]
    squares_x: i32,

    /// 보드 squares Y (기본 7)
    #[arg(long, default_value_t = 7)]
    squares_y: i32,

    /// 체스 칸 한 변 [m] (기본 0.04)
    #[arg(long, default_value_t = 0.04)]
    square_length: f32,

    /// 마커 한 변 [m] (기본 0.02)
    #[arg(long, default_value_t = 0.02)]
    marker_length: f32,
}

fn main() -> Result<()> {
    let args = Args::parse();

    if let Some(path) = args.validate {
        let text =
            fs::read_to_string(&path).with_context(|| format!("읽기 실패: {}", path.display()))?;
        let calib: Calibration = serde_json::from_str(&text)?;
        for cam in &calib.cameras {
            println!(
                "  cam {}: {}x{} fx={:.1} dist_len={}",
                cam.camera_id.index(),
                cam.width,
                cam.height,
                cam.fx,
                cam.dist.len()
            );
        }
        println!(
            "ok: {} cameras, min_triangulation={}",
            calib.camera_count(),
            calib.min_cameras_for_triangulation()
        );
        return Ok(());
    }

    if let Some(n) = args.emit_sim {
        let calib = Calibration::sim(n);
        let json = serde_json::to_string_pretty(&calib)?;
        fs::write(&args.output, json)
            .with_context(|| format!("쓰기 실패: {}", args.output.display()))?;
        println!(
            "wrote sim Calibration ({} cams, dist=[]) → {}",
            n,
            args.output.display()
        );
        return Ok(());
    }

    if let Some(dir) = args.from_images {
        let spec = CharucoBoardSpec {
            squares_x: args.squares_x,
            squares_y: args.squares_y,
            square_length_m: args.square_length,
            marker_length_m: args.marker_length,
        };
        let (calib, report) =
            calibrate_charuco(&dir, spec, CameraId(args.camera_id)).map_err(anyhow::Error::msg)?;
        let json = serde_json::to_string_pretty(&calib)?;
        fs::write(&args.output, json)
            .with_context(|| format!("쓰기 실패: {}", args.output.display()))?;
        println!(
            "wrote ChArUco Calibration → {} (rms={:.4}, frames={}/{})",
            args.output.display(),
            report.rms,
            report.frames_used,
            report.frames_total
        );
        return Ok(());
    }

    anyhow::bail!(
        "사용법: `--emit-sim 3 -o calib.json` 또는 `--validate path` \
         또는 `--from-images DIR -o calib.json`."
    );
}
