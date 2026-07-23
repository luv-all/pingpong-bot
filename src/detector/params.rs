//! 검출 파라미터 타입. 앱 조립 SSOT는 [`crate::defaults`].

use anyhow::{Result, ensure};

/// Scorer hard cuts. `ContourDetector::from(&scorer)`로도 쓴다.
#[derive(Debug, Clone, PartialEq)]
pub struct ScorerParams {
    pub min_area_px: f64,
    pub max_area_px: f64,
    pub min_circularity: f64,
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
