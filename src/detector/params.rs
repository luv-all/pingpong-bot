//! 검출 파라미터 타입. 앱 조립 SSOT는 [`crate::entry`].

use std::fs;
use std::path::Path;

use anyhow::{Context, Result, ensure};
use serde::Deserialize;

use super::{ColorSpace, ColormaskConfig, ColormaskDetector, FuseDetector};

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
    return 80;
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
        return Self {
            generators: default_generators(),
            roi_half_px: default_roi_half_px(),
            cameras: Vec::new(),
            appearance: AppearanceParams::default(),
            scorer: ScorerParams::default(),
            motion: MotionParams::default(),
        };
    }
}

impl Default for AppearanceParams {
    fn default() -> Self {
        return Self {
            colormask: ColormaskParams::default(),
        };
    }
}

impl Default for ColormaskParams {
    fn default() -> Self {
        return Self {
            space: ColorSpace::Ycrcb,
            y_min: 0,
            y_max: 255,
            cr_min: 133,
            cr_max: 173,
            cb_min: 77,
            cb_max: 127,
            h_min: 5,
            h_max: 25,
            s_min: 80,
            s_max: 255,
            v_min: 80,
            v_max: 255,
        };
    }
}

impl Default for ScorerParams {
    fn default() -> Self {
        return Self {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.55,
        };
    }
}

impl Default for MotionParams {
    fn default() -> Self {
        return Self { weight: 0.5 };
    }
}

impl VisionConfig {
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
    let vision = file.vision.unwrap_or_default();
    vision.validate_params()?;
    return Ok(vision);
}

/// TOML `[vision]` → 3레이어 fuse.
pub fn fuse_from_vision(vision: &VisionConfig) -> Result<FuseDetector> {
    return super::dsl::fuse_vision(vision);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_vision_params() {
        let loaded = VisionConfig::default();
        assert_eq!(loaded.generators, vec![Appearance::Colormask]);
        assert_eq!(loaded.roi_half_px, 80);
        assert_eq!(loaded.appearance.colormask.space, ColorSpace::Ycrcb);
        assert_eq!(loaded.scorer.min_circularity, 0.55);
    }
}
