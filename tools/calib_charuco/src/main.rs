//! ChArUco 보드 촬영 → 코너 검출 → 카메라 내부/외부 파라미터 계산 (plan §3.4).
//!
//! 산출물: `Calibration` JSON → 런타임 `--config` / `calibration_path`로 로드.
//! OpenCV 실보정: `cargo run -p calib-charuco --features opencv -- --from-images DIR`

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_infra::Calibration;

#[derive(Parser)]
#[command(name = "calib_charuco", about = "ChArUco 카메라 보정 도구")]
struct Args {
    /// sim 기본 배치 Calibration JSON을 내보낸다 (OpenCV 없이 파이프라인 검증용)
    #[arg(long)]
    emit_sim: Option<u8>,

    /// 출력 JSON 경로
    #[arg(short = 'o', long, default_value = "calibration.json")]
    output: PathBuf,

    /// 기존 Calibration JSON 검증(로드만)
    #[arg(long)]
    validate: Option<PathBuf>,

    /// OpenCV ChArUco 검출 후 Calibration 초안 (`opencv` feature 필요)
    #[arg(long)]
    from_images: Option<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    if let Some(path) = args.validate {
        let text = fs::read_to_string(&path)
            .with_context(|| format!("읽기 실패: {}", path.display()))?;
        let calib: Calibration = serde_json::from_str(&text)?;
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
            "wrote sim Calibration ({} cams) → {}",
            n,
            args.output.display()
        );
        return Ok(());
    }

    if let Some(dir) = args.from_images {
        #[cfg(feature = "opencv")]
        {
            let calib = pingpong_infra::calibrate_charuco_draft(&dir)
                .map_err(anyhow::Error::msg)?;
            let json = serde_json::to_string_pretty(&calib)?;
            fs::write(&args.output, json)
                .with_context(|| format!("쓰기 실패: {}", args.output.display()))?;
            println!("wrote OpenCV ChArUco draft → {}", args.output.display());
            return Ok(());
        }
        #[cfg(not(feature = "opencv"))]
        {
            let _ = dir;
            anyhow::bail!(
                "OpenCV ChArUco는 `--features opencv`로 빌드하세요. \
                 파이프라인 검증은 `--emit-sim 3 -o calib.json`."
            );
        }
    }

    anyhow::bail!(
        "사용법: `--emit-sim 3 -o calib.json` 또는 `--validate path` \
         또는 (`opencv` feature) `--from-images DIR -o calib.json`."
    );
}
