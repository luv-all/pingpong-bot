//! 런타임 TOML 설정 (마일스톤 1.5).

use std::fs;
use std::path::Path;

use anyhow::{Context, Result};
use pingpong_domain::{PhysicsConfig, PhysicsParams};
use pingpong_infra::Calibration;
use serde::Deserialize;

/// `pingpong-bot --config` 로 로드하는 설정.
#[derive(Debug, Clone, Deserialize)]
pub struct RuntimeConfig {
    /// 접수 평면 y [m]
    #[serde(default = "default_hit_plane_y")]
    pub hit_plane_y: f64,
    /// sim 카메라 대수
    #[serde(default = "default_camera_count")]
    pub camera_count: u8,
    /// Calibration JSON 경로 (없으면 sim 레이아웃)
    pub calibration_path: Option<String>,
    /// 로봇 프리셋 id (`pingpong_app::ROBOTS`)
    #[serde(default = "default_robot")]
    pub robot: String,
    /// 물리 계수 — `tools/measure_*`가 `[physics]`에 merge
    #[serde(default)]
    pub physics: PhysicsConfig,
}

fn default_hit_plane_y() -> f64 {
    return pingpong_domain::constants::table::DEFAULT_HIT_PLANE_Y;
}

fn default_camera_count() -> u8 {
    return 3;
}

fn default_robot() -> String {
    return pingpong_app::DEFAULT_ROBOT_ID.into();
}

impl RuntimeConfig {
    /// TOML 파일을 읽는다.
    pub fn load(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path)
            .with_context(|| format!("설정 파일 읽기 실패: {}", path.display()))?;
        let config: Self = toml::from_str(&text)
            .with_context(|| format!("TOML 파싱 실패: {}", path.display()))?;
        return Ok(config);
    }

    /// `[physics]` → concrete [`PhysicsParams`].
    pub fn physics_params(&self) -> PhysicsParams {
        return self.physics.to_params();
    }

    /// Calibration을 로드하거나 sim 기본 배치를 만든다.
    pub fn calibration(&self) -> Result<Calibration> {
        if let Some(ref path) = self.calibration_path {
            let text = fs::read_to_string(path)
                .with_context(|| format!("Calibration 읽기 실패: {path}"))?;
            let calib: Calibration = serde_json::from_str(&text)
                .with_context(|| format!("Calibration JSON 파싱 실패: {path}"))?;
            return Ok(calib);
        }
        return Ok(Calibration::sim(self.camera_count));
    }
}
