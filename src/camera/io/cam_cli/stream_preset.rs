//! 해상도 프리셋.

use clap::ValueEnum;

use crate::constants::camera::arducam_b0332;

/// 해상도 프리셋 — `--width/--height` 대신 대역 실험용.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum StreamPreset {
    /// B0332 네이티브 1280×800
    Full,
    /// 960×600
    Mid,
    /// 640×400 (hinguri 스테레오급)
    Low,
}

impl StreamPreset {
    pub fn size(self) -> (i32, i32) {
        return match self {
            Self::Full => (arducam_b0332::WIDTH, arducam_b0332::HEIGHT),
            Self::Mid => (arducam_b0332::WIDTH_MID, arducam_b0332::HEIGHT_MID),
            Self::Low => (arducam_b0332::WIDTH_LOW, arducam_b0332::HEIGHT_LOW),
        };
    }
}
