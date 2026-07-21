//! 비인터랙티브 모드: `--validate` / `--emit-sim` / `--from-images`.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result};

use crate::args::{Args, board_spec, resolve_output};
use pingpong_bot::{Calibration, CameraId, calibrate_charuco};

pub fn validate(path: &PathBuf) -> Result<()> {
    let text =
        fs::read_to_string(path).with_context(|| format!("읽기 실패: {}", path.display()))?;
    let calib: Calibration = serde_json::from_str(&text)?;
    for cam in &calib.cameras {
        println!(
            "  cam {}: {}x{} fx={:.1} dist_len={}",
            cam.camera_id.0,
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

pub fn emit_sim(n: u8, args: &Args) -> Result<()> {
    let output = resolve_output(args);
    let calib = Calibration::sim(n);
    let json = serde_json::to_string_pretty(&calib)?;
    fs::write(&output, json).with_context(|| format!("쓰기 실패: {}", output.display()))?;
    println!(
        "wrote sim Calibration ({} cams, dist=[]) → {}",
        n,
        output.display()
    );
    return Ok(());
}

pub fn from_images(dir: &PathBuf, args: &Args) -> Result<()> {
    let output = resolve_output(args);
    let (calib, report) = calibrate_charuco(dir, board_spec(args), CameraId(args.camera_id))
        .map_err(anyhow::Error::msg)?;
    let json = serde_json::to_string_pretty(&calib)?;
    fs::write(&output, json).with_context(|| format!("쓰기 실패: {}", output.display()))?;
    println!(
        "wrote ChArUco Calibration → {} (rms={:.4}, frames={}/{})",
        output.display(),
        report.rms,
        report.frames_used,
        report.frames_total
    );
    return Ok(());
}
