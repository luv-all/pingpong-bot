//! 런타임 TOML 설정.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, ensure};
use pingpong_domain::PhysicsParams;
use pingpong_infra::Calibration;
use serde::Deserialize;

/// 기본 런타임 설정 파일.
pub const DEFAULT_CONFIG_PATH: &str = "config/default.toml";

/// 실행 모드.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RuntimeMode {
    Sim,
    Real,
}

/// 시뮬레이터 설정.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct SimConfig {
    pub gui: bool,
    pub frames: u64,
    pub speed: f64,
    pub physics_hz: f64,
    pub frame_hz: f64,
    pub shoot_on_start: bool,
    pub use_ground_truth: bool,
}

/// TOML 하나로 로드하는 전체 런타임 설정.
#[derive(Debug, Clone, Deserialize)]
pub struct RuntimeConfig {
    pub mode: RuntimeMode,
    /// 접수 평면 y [m]
    pub hit_plane_y: f64,
    /// sim 카메라 대수
    pub camera_count: u8,
    /// Calibration JSON 경로 (없으면 sim 레이아웃)
    pub calibration_path: Option<PathBuf>,
    /// 로봇 프리셋 id (`pingpong_app::ROBOTS`)
    pub robot: String,
    /// 커스텀 URDF. 상대 경로는 TOML 파일 기준.
    pub urdf_path: Option<PathBuf>,
    /// 커스텀 URDF 엔드이펙터 링크.
    pub ee_link: Option<String>,
    pub sim: SimConfig,
    /// 물리 계수 — `tools/measure_*`가 `[physics]`에 merge
    pub physics: PhysicsParams,
    /// 상대 asset 경로의 기준 디렉터리.
    #[serde(skip)]
    source_dir: PathBuf,
}

impl RuntimeConfig {
    /// TOML 파일을 읽는다.
    pub fn load(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path)
            .with_context(|| format!("설정 파일 읽기 실패: {}", path.display()))?;
        return Self::from_toml(&text, path);
    }

    fn from_toml(text: &str, path: &Path) -> Result<Self> {
        let mut config: Self =
            toml::from_str(text).with_context(|| format!("TOML 파싱 실패: {}", path.display()))?;
        config.source_dir = path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .to_path_buf();
        config
            .validate()
            .with_context(|| format!("TOML 값 검증 실패: {}", path.display()))?;
        return Ok(config);
    }

    fn validate(&self) -> Result<()> {
        ensure!(self.camera_count >= 2, "camera_count는 2 이상이어야 합니다");
        ensure!(
            self.hit_plane_y.is_finite(),
            "hit_plane_y는 유한해야 합니다"
        );
        ensure!(
            self.sim.speed.is_finite() && self.sim.speed > 0.0,
            "sim.speed는 0보다 커야 합니다"
        );
        ensure!(
            self.sim.physics_hz.is_finite() && self.sim.physics_hz > 0.0,
            "sim.physics_hz는 0보다 커야 합니다"
        );
        ensure!(
            self.sim.frame_hz.is_finite() && self.sim.frame_hz > 0.0,
            "sim.frame_hz는 0보다 커야 합니다"
        );
        ensure!(
            self.physics.restitution.is_finite() && (0.0..=1.0).contains(&self.physics.restitution),
            "physics.restitution은 0..=1이어야 합니다"
        );
        ensure!(
            self.physics.friction.is_finite() && (0.0..=1.0).contains(&self.physics.friction),
            "physics.friction은 0..=1이어야 합니다"
        );
        ensure!(
            self.physics.drag.is_finite() && self.physics.drag >= 0.0,
            "physics.drag는 0 이상이어야 합니다"
        );
        return Ok(());
    }

    /// `[physics]` → concrete [`PhysicsParams`].
    pub fn physics_params(&self) -> PhysicsParams {
        return self.physics;
    }

    /// 설정 파일 위치를 기준으로 Calibration 경로를 해석한다.
    pub fn calibration_path(&self) -> Option<PathBuf> {
        return self
            .calibration_path
            .as_deref()
            .map(|path| self.resolve_path(path));
    }

    /// 설정 파일 위치를 기준으로 커스텀 URDF 경로를 해석한다.
    pub fn urdf_path(&self) -> Option<PathBuf> {
        return self
            .urdf_path
            .as_deref()
            .map(|path| self.resolve_path(path));
    }

    /// Calibration을 로드하거나 sim 기본 배치를 만든다.
    pub fn calibration(&self) -> Result<Calibration> {
        if let Some(path) = self.calibration_path() {
            let text = fs::read_to_string(&path)
                .with_context(|| format!("Calibration 읽기 실패: {}", path.display()))?;
            let calib: Calibration = serde_json::from_str(&text)
                .with_context(|| format!("Calibration JSON 파싱 실패: {}", path.display()))?;
            return Ok(calib);
        }
        return Ok(Calibration::sim(self.camera_count));
    }

    fn resolve_path(&self, path: &Path) -> PathBuf {
        if path.is_absolute() {
            return path.to_path_buf();
        }
        return self.source_dir.join(path);
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;

    const CONFIG: &str = r#"
mode = "sim"
hit_plane_y = 0.30
camera_count = 3
robot = "competition"
calibration_path = "calibration.json"
urdf_path = "robot.urdf"
ee_link = "racket"

[sim]
gui = false
frames = 120
speed = 2.0
physics_hz = 1000.0
frame_hz = 120.0
shoot_on_start = true
use_ground_truth = false

[physics]
restitution = 0.85
friction = 0.15
drag = 0.01
"#;

    #[test]
    fn parses_every_runtime_value_from_toml() {
        let config =
            RuntimeConfig::from_toml(CONFIG, Path::new("config/test.toml")).expect("설정 파싱");

        assert_eq!(config.mode, RuntimeMode::Sim);
        assert_eq!(config.hit_plane_y, 0.30);
        assert_eq!(config.camera_count, 3);
        assert_eq!(config.robot, "competition");
        assert!(!config.sim.gui);
        assert_eq!(config.sim.frames, 120);
        assert_eq!(config.sim.speed, 2.0);
        assert_eq!(config.sim.physics_hz, 1000.0);
        assert_eq!(config.sim.frame_hz, 120.0);
        assert!(config.sim.shoot_on_start);
        assert!(!config.sim.use_ground_truth);
    }

    #[test]
    fn resolves_relative_assets_from_toml_directory() {
        let config =
            RuntimeConfig::from_toml(CONFIG, Path::new("config/test.toml")).expect("설정 파싱");

        assert_eq!(
            config.calibration_path().as_deref(),
            Some(Path::new("config/calibration.json"))
        );
        assert_eq!(
            config.urdf_path().as_deref(),
            Some(Path::new("config/robot.urdf"))
        );
    }

    #[test]
    fn rejects_missing_runtime_fields() {
        let error = RuntimeConfig::from_toml(
            "mode = \"sim\"\n[physics]\nrestitution = 0.85\n",
            Path::new("config/test.toml"),
        )
        .unwrap_err();

        assert!(error.to_string().contains("TOML"));
    }

    #[test]
    fn rejects_invalid_runtime_ranges() {
        let too_few_cameras = CONFIG.replace("camera_count = 3", "camera_count = 1");
        assert!(RuntimeConfig::from_toml(&too_few_cameras, Path::new("config/test.toml")).is_err());

        let negative_speed = CONFIG.replace("speed = 2.0", "speed = -1.0");
        assert!(RuntimeConfig::from_toml(&negative_speed, Path::new("config/test.toml")).is_err());
    }

    #[test]
    fn repository_default_toml_is_complete() {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let config =
            RuntimeConfig::load(&workspace.join(DEFAULT_CONFIG_PATH)).expect("기본 설정 로드");
        assert_eq!(config.mode, RuntimeMode::Sim);
        assert_eq!(config.camera_count, 3);

        let example =
            RuntimeConfig::load(&workspace.join("config/example.toml")).expect("예시 설정 로드");
        assert_eq!(example.mode, RuntimeMode::Sim);
    }
}
