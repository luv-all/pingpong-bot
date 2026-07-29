//! `data/clips/{scene}_{nn}/` 오프라인 클립 경로 해석.

use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use super::rig::CameraRole;

/// `record-stereo` 오프라인 클립 루트.
pub const DEFAULT_CLIPS_DIR: &str = "data/clips";

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

/// `--clip` 해석 결과. `None`이면 라이브.
#[derive(Debug, Clone)]
pub struct ResolvedStereoOffline {
    pub left: PathBuf,
    pub right: PathBuf,
    pub dir: PathBuf,
    pub meas_fps: Option<f64>,
}

impl ResolvedStereoOffline {
    pub fn log(&self) {
        println!(
            "clip {} → {} + {}",
            self.dir.display(),
            self.left.display(),
            self.right.display()
        );
        if let Some(fps) = self.meas_fps {
            println!("clip meas_fps={fps:.2}");
        }
    }

    pub fn paths(&self) -> [PathBuf; 2] {
        return [self.left.clone(), self.right.clone()];
    }
}

#[derive(Debug, Deserialize)]
struct ClipMetaFile {
    meas_fps: Option<f64>,
}

/// `fly_01` → `data/clips/fly_01`, 또는 이미 디렉터리면 그대로.
pub fn resolve_clip_dir(clip: &Path) -> Result<PathBuf, String> {
    if clip.as_os_str().is_empty() {
        return Err("--clip 이 비어 있음".into());
    }
    if clip.is_dir() {
        return Ok(clip.to_path_buf());
    }
    let under_default = PathBuf::from(DEFAULT_CLIPS_DIR).join(clip);
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
        "클립 디렉터리 없음: {} — `{DEFAULT_CLIPS_DIR}/{}` 를 확인",
        clip.display(),
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
pub fn resolve_stereo_clip(clip: &Path) -> Result<StereoClip, String> {
    let dir = resolve_clip_dir(clip)?;
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
pub fn resolve_clip_side(clip: &Path, role: CameraRole) -> Result<PathBuf, String> {
    let dir = resolve_clip_dir(clip)?;
    return find_side_video(&dir, role.as_str());
}

/// `--clip` → 오프라인 경로. 없으면 `Ok(None)` (라이브).
pub fn resolve_stereo_offline(clip: Option<&Path>) -> Result<Option<ResolvedStereoOffline>, String> {
    let Some(clip) = clip else {
        return Ok(None);
    };
    let s = resolve_stereo_clip(clip)?;
    return Ok(Some(ResolvedStereoOffline {
        left: s.left,
        right: s.right,
        dir: s.dir,
        meas_fps: s.meas_fps,
    }));
}

/// `--clip` → 단안 파일. 없으면 `Ok(None)` (라이브).
pub fn resolve_mono_offline(
    clip: Option<&Path>,
    role: CameraRole,
) -> Result<Option<PathBuf>, String> {
    let Some(clip) = clip else {
        return Ok(None);
    };
    return Ok(Some(resolve_clip_side(clip, role)?));
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn resolves_name_under_default_clips() {
        let root = tempfile_dir();
        let clips = root.join("data").join("clips").join("fly_01");
        fs::create_dir_all(&clips).unwrap();
        fs::write(clips.join("left.avi"), b"x").unwrap();
        fs::write(clips.join("right.avi"), b"y").unwrap();
        fs::write(clips.join("meta.json"), r#"{"meas_fps":40.5}"#).unwrap();

        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(&root).unwrap();
        let got = resolve_stereo_clip(Path::new("fly_01")).unwrap();
        let offline = resolve_stereo_offline(Some(Path::new("fly_01")))
            .unwrap()
            .expect("offline");
        std::env::set_current_dir(prev).unwrap();

        assert!(got.left.ends_with("left.avi"));
        assert!(got.right.ends_with("right.avi"));
        assert!((got.meas_fps.unwrap() - 40.5).abs() < 1e-9);
        assert!((offline.meas_fps.unwrap() - 40.5).abs() < 1e-9);
    }

    #[test]
    fn live_when_no_clip() {
        assert!(resolve_stereo_offline(None).unwrap().is_none());
    }

    fn tempfile_dir() -> PathBuf {
        let mut d = std::env::temp_dir();
        d.push(format!(
            "pingpong-clip-test-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir_all(&d).unwrap();
        return d;
    }
}
