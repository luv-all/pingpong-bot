//! `--validate` / `--from-pixels` / merge·저장.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use pingpong_bot::{
    Calibration, CameraId, PixelPoint, calibrate_table_pnp, ensure_reproj_below, upsert_camera,
};
use serde::Deserialize;

use crate::args::{Args, resolve_output};

#[derive(Debug, Deserialize)]
struct PixelsFile {
    width: u32,
    height: u32,
    /// `[[u,v], ...]` 길이 6, `table_landmarks()` 순서
    pixels: Vec<[f64; 2]>,
    #[serde(default)]
    label: Option<String>,
}

pub fn validate(path: &PathBuf) -> Result<()> {
    let text =
        fs::read_to_string(path).with_context(|| format!("읽기 실패: {}", path.display()))?;
    let calib: Calibration = serde_json::from_str(&text)?;
    for cam in &calib.cameras {
        println!(
            "  cam {}: {}x{} fx={:.1} fy={:.1} dist_len={} label={:?}",
            cam.camera_id.0,
            cam.width,
            cam.height,
            cam.fx,
            cam.fy,
            cam.dist.len(),
            cam.label
        );
    }
    println!(
        "ok: {} cameras, min_triangulation={}",
        calib.camera_count(),
        calib.min_cameras_for_triangulation()
    );
    return Ok(());
}

pub fn from_pixels(path: &PathBuf, args: &Args) -> Result<()> {
    let text =
        fs::read_to_string(path).with_context(|| format!("읽기 실패: {}", path.display()))?;
    let file: PixelsFile = serde_json::from_str(&text)
        .with_context(|| format!("pixels JSON: {}", path.display()))?;
    let pixels: Vec<PixelPoint> = file
        .pixels
        .iter()
        .map(|p| PixelPoint::new(p[0], p[1]))
        .collect();
    let result = calibrate_table_pnp(
        CameraId(args.camera_id),
        file.label,
        file.width,
        file.height,
        args.fov_y,
        &pixels,
    )
    .map_err(anyhow::Error::msg)?;
    if result.reproj_rmse > args.max_rmse {
        bail!(
            "재투영 RMSE {:.2} px > --max-rmse {}",
            result.reproj_rmse,
            args.max_rmse
        );
    }
    ensure_reproj_below(&result, args.max_rmse).map_err(anyhow::Error::msg)?;
    return write_result(args, result.params, result.reproj_rmse, result.candidates);
}

pub fn write_result(
    args: &Args,
    params: pingpong_bot::CameraParams,
    rmse: f64,
    candidates: usize,
) -> Result<()> {
    let output = resolve_output(args);
    let mut calib = if let Some(merge) = &args.merge {
        let text = fs::read_to_string(merge)
            .with_context(|| format!("merge 읽기: {}", merge.display()))?;
        serde_json::from_str::<Calibration>(&text)
            .with_context(|| format!("merge JSON: {}", merge.display()))?
    } else if output.exists() && args.merge.is_none() {
        // -o 파일이 이미 있으면 upsert (멀티캠 반복 실행)
        match fs::read_to_string(&output) {
            Ok(text) => serde_json::from_str::<Calibration>(&text).unwrap_or_else(|_| Calibration {
                cameras: Vec::new(),
            }),
            Err(_) => Calibration {
                cameras: Vec::new(),
            },
        }
    } else {
        Calibration {
            cameras: Vec::new(),
        }
    };

    upsert_camera(&mut calib, params);
    let json = serde_json::to_string_pretty(&calib)?;
    fs::write(&output, json).with_context(|| format!("쓰기 실패: {}", output.display()))?;
    println!(
        "wrote table-PnP Calibration → {} (cam={}, rmse={:.2}px, candidates={}, cams={})",
        output.display(),
        args.camera_id,
        rmse,
        candidates,
        calib.camera_count()
    );
    return Ok(());
}
