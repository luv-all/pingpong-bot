//! 머신 로컬 오버레이 — 포트·경로만. 앱 숫자·배선 SSOT는 [`crate::entry`].

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::Deserialize;

/// 기본 로컬 오버레이 경로 (없으면 entry 기본값만 사용).
pub const DEFAULT_LOCAL_PATH: &str = "config/local.toml";

/// 이 머신에서만 다른 값 (시리얼 포트, 캘리브 경로 등).
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
#[serde(default)]
pub struct LocalMachine {
    /// Dynamixel 시리얼 포트. 예: `COM8`, `/dev/tty.usbserial-*`
    pub dxl_port: Option<String>,
    /// Calibration JSON (없으면 sim 레이아웃).
    pub calibration_path: Option<PathBuf>,
}

impl LocalMachine {
    pub fn load(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path)
            .with_context(|| format!("local overlay 읽기 실패: {}", path.display()))?;
        return toml::from_str(&text)
            .with_context(|| format!("local overlay 파싱 실패: {}", path.display()));
    }

    /// 파일이 없으면 `Ok(None)`.
    pub fn load_optional(path: &Path) -> Result<Option<Self>> {
        if !path.exists() {
            return Ok(None);
        }
        return Ok(Some(Self::load(path)?));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_port_only() {
        let local: LocalMachine = toml::from_str(
            r#"
dxl_port = "COM3"
"#,
        )
        .expect("parse");
        assert_eq!(local.dxl_port.as_deref(), Some("COM3"));
    }
}
