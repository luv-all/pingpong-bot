//! 단안 오프라인 입력.

use std::path::PathBuf;

use clap::Parser;

use crate::camera::Role;
use crate::camera::io::clip::resolve_mono_offline;

/// 단안 오프라인 입력 (`--clip`). 없으면 라이브.
#[derive(Parser, Debug, Clone, Default)]
pub struct MonoOfflineArgs {
    /// `data/clips` 클립 (`fly_01`) — `--cam` 쪽 left/right 자동
    #[arg(long, value_name = "NAME|DIR")]
    pub clip: Option<PathBuf>,
}

impl MonoOfflineArgs {
    pub fn resolve(&self, role: Role) -> Result<Option<PathBuf>, String> {
        return resolve_mono_offline(self.clip.as_deref(), role);
    }

    pub fn has_offline(&self) -> bool {
        return self.clip.is_some();
    }
}
