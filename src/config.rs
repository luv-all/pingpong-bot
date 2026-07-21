//! 런타임 TOML 설정.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, ensure};
use pingpong_bot::Calibration;
use pingpong_bot::VisionConfig;
use pingpong_bot::hardware::dynamixel::DynamixelConfig;
use pingpong_bot::hardware::rail::RailConfig;
use pingpong_bot::{InterceptWindow, PhysicsParams};
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

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct InterceptConfig {
    pub y_min: f64,
    pub y_max: f64,
    pub sample_step: f64,
}

impl From<InterceptConfig> for InterceptWindow {
    fn from(c: InterceptConfig) -> Self {
        return InterceptWindow {
            y_min: c.y_min,
            y_max: c.y_max,
            sample_step: c.sample_step,
        };
    }
}

/// 실물 하드웨어 설정. sim에서는 비어 있어도 된다.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct HardwareConfig {
    pub dynamixel: Option<DynamixelConfig>,
    pub rail: Option<RailConfig>,
}

/// TOML 하나로 로드하는 전체 런타임 설정.
#[derive(Debug, Clone, Deserialize)]
pub struct RuntimeConfig {
    pub mode: RuntimeMode,
    pub intercept: InterceptConfig,
    /// sim 카메라 대수
    pub camera_count: u8,
    /// Calibration JSON 경로 (없으면 sim 레이아웃)
    pub calibration_path: Option<PathBuf>,
    /// 로봇 프리셋 id (`pingpong_bot::ROBOTS`)
    pub robot: String,
    /// 커스텀 URDF. 상대 경로는 TOML 파일 기준.
    pub urdf_path: Option<PathBuf>,
    /// 커스텀 URDF 엔드이펙터 링크.
    pub ee_link: Option<String>,
    pub sim: SimConfig,
    #[serde(default)]
    pub hardware: HardwareConfig,
    /// 검출 튜닝(+ optional cameras). 없으면 임베드 `config/default.toml` `[vision]`.
    #[serde(default)]
    pub vision: VisionConfig,
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
        if let Some(rail) = &mut config.hardware.rail
            && rail.enabled
            && rail.dll_path.is_relative()
        {
            rail.dll_path = config.source_dir.join(&rail.dll_path);
        }
        config
            .validate()
            .with_context(|| format!("TOML 값 검증 실패: {}", path.display()))?;
        return Ok(config);
    }

    fn validate(&self) -> Result<()> {
        ensure!(self.camera_count >= 2, "camera_count는 2 이상이어야 합니다");
        ensure!(
            self.intercept.y_min.is_finite()
                && self.intercept.y_max.is_finite()
                && self.intercept.sample_step.is_finite(),
            "intercept 값은 유한해야 합니다"
        );
        ensure!(
            self.intercept.y_min < self.intercept.y_max,
            "intercept.y_min은 y_max보다 작아야 합니다"
        );
        ensure!(
            self.intercept.sample_step > 0.0,
            "intercept.sample_step은 0보다 커야 합니다"
        );
        ensure!(
            !InterceptWindow::from(self.intercept)
                .hit_planes()
                .is_empty(),
            "intercept 후보 수가 너무 많습니다"
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
        self.vision.validate_params().context("vision")?;
        if self.mode == RuntimeMode::Real {
            let dynamixel = self
                .hardware
                .dynamixel
                .as_ref()
                .context("mode=real에는 [hardware.dynamixel] 설정이 필요합니다")?;
            dynamixel.validate().context("hardware.dynamixel")?;
            if let Some(rail) = &self.hardware.rail {
                rail.validate().context("hardware.rail")?;
            }
            if !self.vision.cameras.is_empty() {
                ensure!(
                    self.calibration_path.is_some(),
                    "mode=real + vision.cameras에는 calibration_path가 필요합니다"
                );
                for cam in &self.vision.cameras {
                    ensure!(
                        cam.device.is_some() ^ cam.path.is_some(),
                        "vision.cameras id={} 는 device 또는 path 중 하나만 필요합니다",
                        cam.id
                    );
                }
            }
        }
        return Ok(());
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

    pub(crate) fn resolve_path(&self, path: &Path) -> PathBuf {
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
camera_count = 3
robot = "competition"
calibration_path = "calibration.json"
urdf_path = "robot.urdf"
ee_link = "racket"

[intercept]
y_min = 0.20
y_max = 0.55
sample_step = 0.05

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
        assert_eq!(config.intercept.y_min, 0.20);
        assert_eq!(config.intercept.y_max, 0.55);
        assert_eq!(config.intercept.sample_step, 0.05);
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
    fn resolves_relative_rail_dll_path_from_toml_directory() {
        let real = CONFIG.replace("mode = \"sim\"", "mode = \"real\"")
            + r#"

[hardware.dynamixel]
port = "COM9"
motor_ids = [1, 3, 4, 5]

[hardware.rail]
enabled = true
dll_path = "drivers/AXL.dll"
"#;
        let config =
            RuntimeConfig::from_toml(&real, Path::new("config/real.toml")).expect("real 설정");

        assert_eq!(
            config.hardware.rail.expect("rail 설정").dll_path,
            Path::new("config/drivers/AXL.dll")
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

        let reversed_intercept = CONFIG.replace("y_min = 0.20", "y_min = 0.60");
        assert!(
            RuntimeConfig::from_toml(&reversed_intercept, Path::new("config/test.toml")).is_err()
        );

        let too_many_samples = CONFIG.replace("sample_step = 0.05", "sample_step = 1e-20");
        assert!(
            RuntimeConfig::from_toml(&too_many_samples, Path::new("config/test.toml")).is_err()
        );
    }

    #[test]
    fn rejects_invalid_rail_range_when_enabled() {
        let real = CONFIG.replace("mode = \"sim\"", "mode = \"real\"")
            + r#"

[hardware.dynamixel]
port = "COM9"
motor_ids = [1, 3, 4, 5]

[hardware.rail]
enabled = true
dll_path = "drivers/AXL.dll"
x_min_m = 0.50
x_max_m = 0.20
"#;
        let error =
            RuntimeConfig::from_toml(&real, Path::new("config/real.toml")).unwrap_err();
        assert!(format!("{error:#}").contains("hardware.rail"));
    }

    #[test]
    fn real_mode_requires_and_parses_dynamixel_config() {
        let real = CONFIG.replace("mode = \"sim\"", "mode = \"real\"")
            + r#"

[hardware.dynamixel]
port = "COM9"
motor_ids = [1, 3, 4, 5]
"#;
        let config =
            RuntimeConfig::from_toml(&real, Path::new("config/real.toml")).expect("real 설정");
        let dynamixel = config.hardware.dynamixel.as_ref().expect("Dynamixel 설정");
        assert_eq!(dynamixel.port, "COM9");
        assert_eq!(dynamixel.motor_ids, [1, 3, 4, 5]);

        let missing = CONFIG.replace("mode = \"sim\"", "mode = \"real\"");
        let error = RuntimeConfig::from_toml(&missing, Path::new("config/real.toml")).unwrap_err();
        assert!(format!("{error:#}").contains("hardware.dynamixel"));
    }

    #[test]
    fn repository_default_toml_is_complete() {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).to_path_buf();
        let config =
            RuntimeConfig::load(&workspace.join(DEFAULT_CONFIG_PATH)).expect("기본 설정 로드");
        assert_eq!(config.mode, RuntimeMode::Sim);
        assert_eq!(config.camera_count, 3);

        let example =
            RuntimeConfig::load(&workspace.join("config/example.toml")).expect("예시 설정 로드");
        assert_eq!(example.mode, RuntimeMode::Sim);

        let real = RuntimeConfig::load(&workspace.join("config/real-hardware.toml"))
            .expect("real 하드웨어 설정 로드");
        assert_eq!(real.mode, RuntimeMode::Real);
        assert_eq!(
            real.hardware
                .dynamixel
                .as_ref()
                .expect("Dynamixel")
                .motor_ids,
            [1, 3, 4, 5]
        );
    }
}
