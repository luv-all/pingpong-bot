//! 스테레오 오프라인 입력.

use std::path::PathBuf;

use clap::Parser;

use crate::camera::io::clip::{ResolvedStereoOffline, resolve_stereo_offline};

/// 스테레오 오프라인 입력 (`--clip`). 없으면 라이브.
#[derive(Parser, Debug, Clone, Default)]
pub struct StereoOfflineArgs {
    /// `data/clips` 클립 이름 또는 디렉터리 (`fly_01`)
    #[arg(long, value_name = "NAME|DIR")]
    pub clip: Option<PathBuf>,
}

impl StereoOfflineArgs {
    pub fn resolve(&self) -> Result<Option<ResolvedStereoOffline>, String> {
        return resolve_stereo_offline(self.clip.as_deref());
    }

    pub fn has_offline(&self) -> bool {
        return self.clip.is_some();
    }
}
