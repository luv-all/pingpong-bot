//! `--clip` 해석 결과.

use std::path::{Path, PathBuf};

use super::stereo_clip::{resolve_clip_side, resolve_stereo_clip};
use crate::camera::Role;

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

/// `--clip` → 오프라인 경로. 없으면 `Ok(None)` (라이브).
pub(crate) fn resolve_stereo_offline(
    clip: Option<&Path>,
) -> Result<Option<ResolvedStereoOffline>, String> {
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
pub(crate) fn resolve_mono_offline(
    clip: Option<&Path>,
    role: Role,
) -> Result<Option<PathBuf>, String> {
    let Some(clip) = clip else {
        return Ok(None);
    };
    return Ok(Some(resolve_clip_side(clip, role)?));
}
