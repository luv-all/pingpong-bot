//! `--validate` / `--from-pixels` / merge·저장 · pending 사이드카.

use std::fs;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use pingpong_bot::camera::{self, Calibration, TablePnp};
use serde::Deserialize;

use crate::args::{Args, pending_path, resolve_camera_id, resolve_output};

#[derive(Debug, Deserialize)]
struct PixelsFile {
    width: u32,
    height: u32,
    /// `[[u,v], ...]` 길이 8, `table_landmarks()` 순서
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
    let file: PixelsFile =
        serde_json::from_str(&text).with_context(|| format!("pixels JSON: {}", path.display()))?;
    let pixels: Vec<camera::Pixel> = file
        .pixels
        .iter()
        .map(|p| camera::Pixel::new(p[0], p[1]))
        .collect();
    let result = TablePnp::calibrate(
        resolve_camera_id(args).map_err(anyhow::Error::msg)?,
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
    TablePnp::ensure_reproj_below(&result, args.max_rmse).map_err(anyhow::Error::msg)?;
    return write_result(args, result.params, result.reproj_rmse, result.candidates);
}

/// 본파일(`-o`) 또는 `--merge`에서 이 카메라 params. 없으면 `None`.
pub fn load_baseline_params(args: &Args, cam_id: camera::Id) -> Option<camera::Params> {
    let path = args.merge.as_ref().unwrap_or(&args.output);
    let text = fs::read_to_string(path).ok()?;
    let calib: Calibration = serde_json::from_str(&text).ok()?;
    return calib.params(cam_id).cloned();
}

/// 시작 시 pending이 있으면 안내 (본파일은 안 건드림).
pub fn hint_pending_if_exists(args: &Args, cam_id: camera::Id) {
    let path = pending_path(args);
    let Some(calib) = read_pending_file(&path) else {
        return;
    };
    if calib.cameras.is_empty() {
        return;
    }
    let ids: Vec<String> = calib
        .cameras
        .iter()
        .map(|c| c.camera_id.0.to_string())
        .collect();
    let has_this = calib.params(cam_id).is_some();
    println!(
        "pending exists: {} (cams=[{}]) — s promotes cam{} → {}, or ignore",
        path.display(),
        ids.join(","),
        cam_id.0,
        resolve_output(args).display()
    );
    if !has_this {
        println!("  (this session cam{} not in pending yet)", cam_id.0);
    }
}

fn read_pending_file(path: &std::path::Path) -> Option<Calibration> {
    if !path.is_file() {
        return None;
    }
    let text = fs::read_to_string(path).ok()?;
    return serde_json::from_str(&text).ok();
}

/// accepted 해를 공유 pending 번들에 upsert (`-o` / merge 미변경).
pub fn write_pending(
    args: &Args,
    params: camera::Params,
    rmse: f64,
    candidates: usize,
) -> Result<PathBuf> {
    let cam_id = params.camera_id;
    let path = pending_path(args);
    let mut calib = read_pending_file(&path).unwrap_or_else(|| Calibration {
        cameras: Vec::new(),
    });
    TablePnp::upsert_camera(&mut calib, params);
    let json = serde_json::to_string_pretty(&calib)?;
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)
                .with_context(|| format!("디렉터리 생성: {}", parent.display()))?;
        }
    }
    fs::write(&path, json).with_context(|| format!("pending 쓰기: {}", path.display()))?;
    println!(
        "pending upsert → {} (cam={}, rmse={:.2}px, candidates={}, pending_cams={}) — s=promote, q=keep",
        path.display(),
        cam_id.0,
        rmse,
        candidates,
        calib.camera_count()
    );
    return Ok(path);
}

/// pending 배열에서 해당 카메라만 제거. 비면 파일 삭제.
pub fn clear_pending_camera(args: &Args, cam_id: camera::Id) {
    let path = pending_path(args);
    let Some(mut calib) = read_pending_file(&path) else {
        return;
    };
    let before = calib.cameras.len();
    calib.cameras.retain(|c| c.camera_id != cam_id);
    if calib.cameras.len() == before {
        return;
    }
    if calib.cameras.is_empty() {
        match fs::remove_file(&path) {
            Ok(()) => println!("cleared pending {}", path.display()),
            Err(e) => eprintln!("pending 삭제 실패 {}: {e}", path.display()),
        }
        return;
    }
    match serde_json::to_string_pretty(&calib) {
        Ok(json) => match fs::write(&path, json) {
            Ok(()) => println!(
                "pending removed cam{} → {} ({} left)",
                cam_id.0,
                path.display(),
                calib.camera_count()
            ),
            Err(e) => eprintln!("pending 쓰기 실패 {}: {e}", path.display()),
        },
        Err(e) => eprintln!("pending serialize 실패 {}: {e}", path.display()),
    }
}

/// pending 파일에 이 카메라가 있으면 true.
pub fn pending_has_camera(args: &Args, cam_id: camera::Id) -> bool {
    return read_pending_file(&pending_path(args))
        .map(|c| c.params(cam_id).is_some())
        .unwrap_or(false);
}

/// 디스크 pending → 본파일 promote (재클릭 없이 `s`).
pub fn promote_pending(args: &Args, cam_id: camera::Id) -> Result<()> {
    let path = pending_path(args);
    let text =
        fs::read_to_string(&path).with_context(|| format!("pending 읽기: {}", path.display()))?;
    let calib: Calibration =
        serde_json::from_str(&text).with_context(|| format!("pending JSON: {}", path.display()))?;
    let Some(params) = calib.params(cam_id).cloned() else {
        bail!("pending {} 에 cam {} 없음", path.display(), cam_id.0);
    };
    let rmse = rmse_from_label(params.label.as_deref()).unwrap_or(0.0);
    return write_result(args, params, rmse, 0);
}

fn rmse_from_label(label: Option<&str>) -> Option<f64> {
    let label = label?;
    let rest = label.strip_prefix("table-pnp rmse=")?;
    let num = rest.split("px").next()?;
    return num.parse().ok();
}

pub fn write_result(
    args: &Args,
    params: camera::Params,
    rmse: f64,
    candidates: usize,
) -> Result<()> {
    let cam_id = params.camera_id;
    let output = resolve_output(args);
    let mut calib = if let Some(merge) = &args.merge {
        let text = fs::read_to_string(merge)
            .with_context(|| format!("merge 읽기: {}", merge.display()))?;
        serde_json::from_str::<Calibration>(&text)
            .with_context(|| format!("merge JSON: {}", merge.display()))?
    } else if output.exists() && args.merge.is_none() {
        // -o 파일이 이미 있으면 upsert (멀티캠 반복 실행)
        match fs::read_to_string(&output) {
            Ok(text) => {
                serde_json::from_str::<Calibration>(&text).unwrap_or_else(|_| Calibration {
                    cameras: Vec::new(),
                })
            }
            Err(_) => Calibration {
                cameras: Vec::new(),
            },
        }
    } else {
        Calibration {
            cameras: Vec::new(),
        }
    };

    TablePnp::upsert_camera(&mut calib, params);
    let json = serde_json::to_string_pretty(&calib)?;
    if let Some(parent) = output.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)
                .with_context(|| format!("디렉터리 생성: {}", parent.display()))?;
        }
    }
    fs::write(&output, json).with_context(|| format!("쓰기 실패: {}", output.display()))?;
    clear_pending_camera(args, cam_id);
    println!(
        "wrote table-PnP Calibration → {} (cam={}, rmse={:.2}px, candidates={}, cams={})",
        output.display(),
        cam_id.0,
        rmse,
        candidates,
        calib.camera_count()
    );
    return Ok(());
}
