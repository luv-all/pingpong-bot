//! 실기 레일 홈잉 결과 — 재빌드 없이 `RailConfig`의 영점을 덮어쓰는 사이드카 JSON.
//!
//! `data/calibration.json`(카메라 PnP)과 같은 자리·패턴. 파일이 없거나 파싱에
//! 실패하면 `None`을 반환해, 호출부가 하드코딩 기본값(`defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M`)
//! 그대로 계속 진행할 수 있게 한다 — 캘리브레이션 파일 문제로 로봇이 못 뜨면 안 된다.

use std::path::Path;

use serde::{Deserialize, Serialize};

use super::rail_config::{RailConfig, RailEnd};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
enum RailEndJson {
    Min,
    Max,
}

impl From<RailEnd> for RailEndJson {
    fn from(end: RailEnd) -> Self {
        return match end {
            RailEnd::Min => RailEndJson::Min,
            RailEnd::Max => RailEndJson::Max,
        };
    }
}

impl From<RailEndJson> for RailEnd {
    fn from(end: RailEndJson) -> Self {
        return match end {
            RailEndJson::Min => RailEnd::Min,
            RailEndJson::Max => RailEnd::Max,
        };
    }
}

/// `--calibrate-rail` 홈잉 1회 실행 결과.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RailCalibration {
    pub board_zero_domain_m: f64,
    homed_at_end: RailEndJson,
    pub board_position_at_home_m: f64,
    pub measured_unix_secs: u64,
}

impl RailCalibration {
    pub fn from_home(
        end: RailEnd,
        board_position_at_home_m: f64,
        board_zero_domain_m: f64,
        measured_unix_secs: u64,
    ) -> Self {
        return Self {
            board_zero_domain_m,
            homed_at_end: end.into(),
            board_position_at_home_m,
            measured_unix_secs,
        };
    }

    pub fn homed_at_end(&self) -> RailEnd {
        return self.homed_at_end.into();
    }

    /// 파일이 없거나 파싱에 실패하면 `None` — 호출부는 하드코딩 기본값을 쓴다.
    pub fn load(path: &Path) -> Option<Self> {
        let contents = std::fs::read_to_string(path).ok()?;
        return match serde_json::from_str::<Self>(&contents) {
            Ok(calibration) => Some(calibration),
            Err(error) => {
                tracing::warn!(
                    path = %path.display(),
                    %error,
                    "rail_calibration.json 파싱 실패 — 기본값 사용"
                );
                None
            }
        };
    }

    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let json = serde_json::to_string_pretty(self)
            .expect("RailCalibration 직렬화는 실패할 수 없다 — 모든 필드가 유한 f64/열거형");
        return std::fs::write(path, json);
    }

    /// `config.board_zero_domain_m`만 덮어쓴다. 나머지 필드(범위·속도 등)는 손대지
    /// 않는다 — 홈잉은 영점만 바꾸는 절차다.
    pub fn apply_to(&self, config: &mut RailConfig) {
        config.board_zero_domain_m = self.board_zero_domain_m;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch_path(name: &str) -> std::path::PathBuf {
        return std::env::temp_dir().join(format!(
            "pingpong_bot_rail_calibration_test_{}_{name}.json",
            std::process::id()
        ));
    }

    #[test]
    fn load_returns_none_when_file_missing() {
        let path = scratch_path("missing");
        let _ = std::fs::remove_file(&path);
        assert_eq!(RailCalibration::load(&path), None);
    }

    #[test]
    fn load_returns_none_on_malformed_json() {
        let path = scratch_path("malformed");
        std::fs::write(&path, "not json").unwrap();
        assert_eq!(RailCalibration::load(&path), None);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn save_then_load_round_trips() {
        let path = scratch_path("roundtrip");
        let calibration = RailCalibration::from_home(RailEnd::Min, 0.0, 0.7050, 1_786_412_345);
        calibration.save(&path).unwrap();
        let loaded = RailCalibration::load(&path).unwrap();
        assert_eq!(loaded, calibration);
        assert_eq!(loaded.homed_at_end(), RailEnd::Min);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn apply_to_only_overwrites_board_zero_domain_m() {
        let mut config = RailConfig {
            board_zero_domain_m: 0.0,
            x_min_m: 0.01,
            x_max_m: 1.3395,
            ..RailConfig::default()
        };
        let calibration = RailCalibration::from_home(RailEnd::Max, 1.41, 0.7050, 1_786_412_345);
        calibration.apply_to(&mut config);
        assert_eq!(config.board_zero_domain_m, 0.7050);
        assert_eq!(config.x_min_m, 0.01);
        assert_eq!(config.x_max_m, 1.3395);
    }
}
