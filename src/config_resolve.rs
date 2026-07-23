//! 경로 해석 헬퍼 (도구 공용). 앱 배선 SSOT는 [`crate::entry`].

use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;

/// 측정·검출 도구용 선택 오버레이. 앱은 `config/local.toml` / CLI만 쓴다.
pub const DEFAULT_CONFIG_PATH: &str = "config/tools.toml";

#[derive(Debug, Deserialize)]
struct CalibrationPathField {
    #[serde(default)]
    calibration_path: Option<PathBuf>,
}

/// TOML의 `calibration_path`를 설정 파일 기준 절대/해석 경로로 돌린다.
pub fn calibration_path_from_config(config_path: &Path) -> Result<Option<PathBuf>, String> {
    let text = fs::read_to_string(config_path)
        .map_err(|e| format!("config 읽기 {}: {e}", config_path.display()))?;
    let partial: CalibrationPathField =
        toml::from_str(&text).map_err(|e| format!("config 파싱 {}: {e}", config_path.display()))?;
    let Some(rel) = partial.calibration_path else {
        return Ok(None);
    };
    if rel.is_absolute() {
        return Ok(Some(rel));
    }
    let base = config_path.parent().unwrap_or_else(|| Path::new("."));
    return Ok(Some(base.join(rel)));
}

/// CLI `--calibration`이 있으면 그걸, 없으면 config TOML의 `calibration_path`.
pub fn resolve_calibration_path(
    config_path: &Path,
    cli_override: Option<PathBuf>,
) -> Result<PathBuf, String> {
    if let Some(path) = cli_override {
        return Ok(path);
    }
    return calibration_path_from_config(config_path)?.ok_or_else(|| {
        format!(
            "`{}`에 calibration_path가 없습니다. TOML에 쓰거나 --calibration PATH를 주세요.",
            config_path.display()
        )
    });
}
