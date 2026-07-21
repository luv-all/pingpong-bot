//! 검출 튜닝 SSOT — `config/default.toml` `[vision]`.
//!
//! 레이어: `vision.appearance.*` generators · `vision.scorer`(hard cuts) ·
//! `vision.motion.weight`(soft boost). Rust nesting == TOML nesting.

use std::fs;
use std::path::Path;
use std::sync::OnceLock;

use anyhow::{Context, Result, ensure};
use serde::Deserialize;

use super::{ColorSpace, ColormaskConfig, ColormaskDetector, FuseDetector};

/// 컴파일 시점에 박아 둔 기본 TOML (워크스페이스 `config/default.toml`).
const EMBEDDED_DEFAULT_TOML: &str = include_str!("../../config/default.toml");

/// Appearance generator 종류 (fuse 1층). scorer/motion이 아님.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Default,
    clap::ValueEnum,
    serde::Serialize,
    serde::Deserialize,
)]
#[serde(rename_all = "lowercase")]
#[value(rename_all = "lower")]
pub enum Appearance {
    #[default]
    Colormask,
    Contour,
}

impl std::str::FromStr for Appearance {
    type Err = ParseAppearanceError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        return match s {
            "colormask" => Ok(Self::Colormask),
            "contour" => Ok(Self::Contour),
            _ => Err(ParseAppearanceError),
        };
    }
}

impl std::fmt::Display for Appearance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(match self {
            Self::Colormask => "colormask",
            Self::Contour => "contour",
        });
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParseAppearanceError;

impl std::fmt::Display for ParseAppearanceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str("expected colormask|contour");
    }
}

impl std::error::Error for ParseAppearanceError {}

/// `[vision]` — generators · ROI · 레이어별 파라미터. `[[vision.cameras]]`는 real 전용.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct VisionConfig {
    /// Appearance generator 순서. fuse `FirstSurviving`에 그대로 쓴다.
    #[serde(default = "default_generators")]
    pub generators: Vec<Appearance>,
    /// ROI 반변(px). `track(inner, roi_half_px)`.
    #[serde(default = "default_roi_half_px")]
    pub roi_half_px: i32,
    #[serde(default)]
    pub cameras: Vec<VisionCameraConfig>,
    /// `[vision.appearance.*]` — appearance generator별 파라미터.
    #[serde(default)]
    pub appearance: AppearanceParams,
    /// Scorer hard cuts (area · circularity). Canny generator에도 동일 수치.
    #[serde(default)]
    pub scorer: ScorerParams,
    /// `weight` → MotionPrior soft score. `0`이면 motion prior 비활성.
    #[serde(default)]
    pub motion: MotionParams,
}

fn default_generators() -> Vec<Appearance> {
    return vec![Appearance::Colormask];
}

fn default_roi_half_px() -> i32 {
    return VisionConfig::from_embedded().roi_half_px;
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct VisionCameraConfig {
    pub id: u8,
    pub device: Option<i32>,
    pub path: Option<std::path::PathBuf>,
}

/// `[vision.appearance]` — appearance generator별 파라미터를 nest.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct AppearanceParams {
    #[serde(default)]
    pub colormask: ColormaskParams,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ColormaskParams {
    pub space: ColorSpace,
    pub y_min: u8,
    pub y_max: u8,
    pub cr_min: u8,
    pub cr_max: u8,
    pub cb_min: u8,
    pub cb_max: u8,
    pub h_min: u8,
    pub h_max: u8,
    pub s_min: u8,
    pub s_max: u8,
    pub v_min: u8,
    pub v_max: u8,
    pub min_area_px: f64,
    pub max_area_px: f64,
}

/// `[vision.scorer]` — hard cuts. `ContourDetector::from(&scorer)`로도 쓴다.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ScorerParams {
    pub min_area_px: f64,
    pub max_area_px: f64,
    pub min_circularity: f64,
}

/// `[vision.motion]` — fuse soft motion boost. `0`이면 prior 비활성.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct MotionParams {
    #[serde(default = "default_motion_weight")]
    pub weight: f64,
}

fn default_motion_weight() -> f64 {
    0.5
}

impl Default for VisionConfig {
    fn default() -> Self {
        return Self::from_embedded();
    }
}

impl Default for AppearanceParams {
    fn default() -> Self {
        return VisionConfig::from_embedded().appearance;
    }
}

impl Default for ColormaskParams {
    fn default() -> Self {
        return VisionConfig::from_embedded().appearance.colormask;
    }
}

impl Default for ScorerParams {
    fn default() -> Self {
        return VisionConfig::from_embedded().scorer;
    }
}

impl Default for MotionParams {
    fn default() -> Self {
        return VisionConfig::from_embedded().motion;
    }
}

impl VisionConfig {
    /// `config/default.toml` 임베드본. 테스트·편의 factory용.
    pub fn from_embedded() -> Self {
        static ONCE: OnceLock<VisionConfig> = OnceLock::new();
        return ONCE
            .get_or_init(|| {
                parse_vision_required(EMBEDDED_DEFAULT_TOML)
                    .expect("embedded config/default.toml [vision] must be complete")
            })
            .clone();
    }

    pub fn validate_params(&self) -> Result<()> {
        ensure!(
            !self.generators.is_empty(),
            "vision.generators는 비어 있으면 안 됩니다"
        );
        ensure!(
            self.roi_half_px > 0,
            "vision.roi_half_px는 0보다 커야 합니다"
        );
        ColormaskConfig::try_from(&self.appearance.colormask).context("vision.appearance.colormask")?;
        ensure!(
            self.scorer.min_area_px > 0.0 && self.scorer.max_area_px > self.scorer.min_area_px,
            "vision.scorer area 범위가 잘못됐습니다"
        );
        ensure!(
            (0.0..=1.0).contains(&self.scorer.min_circularity),
            "vision.scorer.min_circularity는 0..=1이어야 합니다"
        );
        ensure!(
            self.motion.weight >= 0.0,
            "vision.motion.weight는 0 이상이어야 합니다"
        );
        return Ok(());
    }
}

impl TryFrom<&ColormaskParams> for ColormaskConfig {
    type Error = anyhow::Error;

    fn try_from(p: &ColormaskParams) -> Result<Self> {
        let (c0_min, c0_max, c1_min, c1_max, c2_min, c2_max) = match p.space {
            ColorSpace::Ycrcb => (p.y_min, p.y_max, p.cr_min, p.cr_max, p.cb_min, p.cb_max),
            ColorSpace::Hsv => (p.h_min, p.h_max, p.s_min, p.s_max, p.v_min, p.v_max),
        };
        return Ok(Self {
            space: p.space,
            c0_min,
            c0_max,
            c1_min,
            c1_max,
            c2_min,
            c2_max,
            min_area_px: p.min_area_px,
            max_area_px: p.max_area_px,
        });
    }
}

impl TryFrom<&ColormaskParams> for ColormaskDetector {
    type Error = anyhow::Error;

    fn try_from(p: &ColormaskParams) -> Result<Self> {
        return Ok(Self::new(ColormaskConfig::try_from(p)?));
    }
}

/// 런타임/툴 설정 파일에서 `[vision]`을 읽는다. 없으면 임베드 기본값.
pub fn load_vision_from_config(path: impl AsRef<Path>) -> Result<VisionConfig> {
    let path = path.as_ref();
    let text = fs::read_to_string(path)
        .with_context(|| format!("설정 파일 읽기 실패: {}", path.display()))?;
    return vision_from_toml(&text)
        .with_context(|| format!("vision 파싱 실패: {}", path.display()));
}

pub fn vision_from_toml(text: &str) -> Result<VisionConfig> {
    #[derive(Deserialize)]
    struct File {
        #[serde(default)]
        vision: Option<VisionConfig>,
    }
    let file: File = toml::from_str(text).context("TOML 파싱")?;
    let vision = file.vision.unwrap_or_else(VisionConfig::from_embedded);
    vision.validate_params()?;
    return Ok(vision);
}

/// 임베드 파싱 — 필드 누락 시 즉시 실패 (Default 재귀 방지).
fn parse_vision_required(text: &str) -> Result<VisionConfig> {
    #[derive(Deserialize)]
    struct File {
        vision: VisionRequired,
    }
    #[derive(Deserialize)]
    struct VisionRequired {
        generators: Vec<Appearance>,
        roi_half_px: i32,
        #[serde(default)]
        cameras: Vec<VisionCameraConfig>,
        appearance: AppearanceParamsRequired,
        scorer: ScorerParams,
        motion: MotionParams,
    }
    #[derive(Deserialize)]
    struct AppearanceParamsRequired {
        colormask: ColormaskParams,
    }
    let file: File = toml::from_str(text).context("embedded TOML")?;
    let v = file.vision;
    return Ok(VisionConfig {
        generators: v.generators,
        roi_half_px: v.roi_half_px,
        cameras: v.cameras,
        appearance: AppearanceParams {
            colormask: v.appearance.colormask,
        },
        scorer: v.scorer,
        motion: v.motion,
    });
}

/// TOML `[vision]` → 3레이어 fuse. 구현은 [`super::dsl::fuse_vision`] (DSL SSOT).
pub fn fuse_from_vision(vision: &VisionConfig) -> Result<FuseDetector> {
    return super::dsl::fuse_vision(vision);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_resolve::DEFAULT_CONFIG_PATH;

    #[test]
    fn embedded_matches_workspace_default_toml() {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR"));
        let loaded = load_vision_from_config(workspace.join(DEFAULT_CONFIG_PATH)).unwrap();
        assert_eq!(loaded, VisionConfig::from_embedded());
        assert_eq!(loaded.generators, vec![Appearance::Colormask]);
        assert_eq!(loaded.roi_half_px, 80);
        assert_eq!(loaded.appearance.colormask.space, ColorSpace::Ycrcb);
        assert_eq!(loaded.scorer.min_circularity, 0.55);
    }
}
