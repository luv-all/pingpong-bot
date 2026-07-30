//! 클립 `meta.json` 스키마.

use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub(super) struct ClipMetaFile {
    pub meas_fps: Option<f64>,
}
