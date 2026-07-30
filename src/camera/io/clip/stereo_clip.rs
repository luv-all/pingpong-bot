//! 스테레오 클립 한 세트.

use std::fs;
use std::path::{Path, PathBuf};

use crate::defaults::DEFAULT_CLIPS_DIR;

use super::clip_meta_file::ClipMetaFile;

const VIDEO_EXTS: &[&str] = &["avi", "mp4", "mkv", "mov"];

/// 스테레오 클립 한 세트 (`left.*` + `right.*` + optional `meta.json`).
#[derive(Debug, Clone)]
pub struct StereoClip {
    pub dir: PathBuf,
    pub left: PathBuf,
    pub right: PathBuf,
    /// `meta.json`의 `meas_fps` (있으면).
    pub meas_fps: Option<f64>,
}

/// `fly_01` → `data/clips/fly_01`, 또는 이미 디렉터리면 그대로.
pub(crate) fn resolve_clip_dir(clip: &Path) -> Result<PathBuf, String> {
    return resolve_clip_dir_under(clip, Path::new(DEFAULT_CLIPS_DIR));
}

/// `resolve_clip_dir`의 루트 주입 버전 — 테스트가 프로세스 CWD를 바꾸지 않도록.
fn resolve_clip_dir_under(clip: &Path, clips_root: &Path) -> Result<PathBuf, String> {
    if clip.as_os_str().is_empty() {
        return Err("--clip 이 비어 있음".into());
    }
    if clip.is_dir() {
        return Ok(clip.to_path_buf());
    }
    let under_default = clips_root.join(clip);
    if under_default.is_dir() {
        return Ok(under_default);
    }
    if clip.components().count() > 1 {
        return Err(format!(
            "클립 디렉터리 없음: {} (또는 {})",
            clip.display(),
            under_default.display()
        ));
    }
    return Err(format!(
        "클립 디렉터리 없음: {} — `{}/{}` 를 확인",
        clip.display(),
        clips_root.display(),
        clip.display()
    ));
}

fn find_side_video(dir: &Path, side: &str) -> Result<PathBuf, String> {
    for ext in VIDEO_EXTS {
        let p = dir.join(format!("{side}.{ext}"));
        if p.is_file() {
            return Ok(p);
        }
    }
    return Err(format!(
        "{}: {side}.({}) 없음",
        dir.display(),
        VIDEO_EXTS.join("|")
    ));
}

fn load_meas_fps(dir: &Path) -> Option<f64> {
    let path = dir.join("meta.json");
    let text = fs::read_to_string(path).ok()?;
    let meta: ClipMetaFile = serde_json::from_str(&text).ok()?;
    let fps = meta.meas_fps?;
    if fps.is_finite() && fps > 1.0 {
        return Some(fps);
    }
    return None;
}

/// `--clip fly_01` → left/right 영상 + optional meas_fps.
pub(crate) fn resolve_stereo_clip(clip: &Path) -> Result<StereoClip, String> {
    return resolve_stereo_clip_under(clip, Path::new(DEFAULT_CLIPS_DIR));
}

/// `resolve_stereo_clip`의 루트 주입 버전 — 테스트가 프로세스 CWD를 바꾸지 않도록.
pub(crate) fn resolve_stereo_clip_under(
    clip: &Path,
    clips_root: &Path,
) -> Result<StereoClip, String> {
    let dir = resolve_clip_dir_under(clip, clips_root)?;
    let left = find_side_video(&dir, "left")?;
    let right = find_side_video(&dir, "right")?;
    let meas_fps = load_meas_fps(&dir);
    return Ok(StereoClip {
        dir,
        left,
        right,
        meas_fps,
    });
}

/// 단안 툴: `--clip fly_01 --cam left` → `…/left.avi`.
pub(crate) fn resolve_clip_side(clip: &Path, role: crate::camera::Role) -> Result<PathBuf, String> {
    let dir = resolve_clip_dir(clip)?;
    return find_side_video(&dir, role.as_str());
}
