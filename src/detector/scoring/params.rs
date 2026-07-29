//! Scorer hard-cut 파라미터. 앱 [`Default`]는 [`crate::defaults::vision`].

use anyhow::{Result, ensure};

/// Scorer hard cuts. `ContourDetector::from(&scorer)`로도 쓴다.
#[derive(Debug, Clone, PartialEq)]
pub struct ScorerParams {
    pub min_area_px: f64,
    pub max_area_px: f64,
    pub min_circularity: f64,
}

impl ScorerParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.min_area_px > 0.0 && self.max_area_px > self.min_area_px,
            "scorer area 범위가 잘못됐습니다"
        );
        ensure!(
            (0.0..=1.0).contains(&self.min_circularity),
            "scorer.min_circularity는 0..=1이어야 합니다"
        );
        return Ok(());
    }

    /// 캘리브 카메라 파라미터로 면적 밴드를 채운다.
    pub fn from_calib(params: &crate::camera::CameraParams, circularity: f64) -> Result<Self> {
        return crate::detector::scorer_params_from_calib(params, circularity);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_scorer_params() {
        let p = ScorerParams::default();
        assert_eq!(p.min_circularity, 0.55);
        assert!(p.validate().is_ok());
    }
}
