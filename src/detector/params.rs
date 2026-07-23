//! 검출 파라미터 타입. 앱 조립 SSOT는 [`crate::defaults`].

use anyhow::{Result, ensure};

/// Adaptive ROI: `half = clamp(k·√(area/π) + pad + m·Δ, half_min, half_max)`.
#[derive(Debug, Clone, PartialEq)]
pub struct RoiParams {
    /// 등가 반경 배율.
    pub k: f64,
    /// 고정 여유 [px].
    pub pad: i32,
    /// 직전 프레임 이동량 `|Δpx|` 배율.
    pub m: f64,
    pub half_min: i32,
    pub half_max: i32,
}

impl Default for RoiParams {
    fn default() -> Self {
        return Self {
            k: 3.5,
            pad: 24,
            m: 1.0,
            half_min: 48,
            half_max: 240,
        };
    }
}

impl From<i32> for RoiParams {
    /// 고정 half — adaptive 끔 (`k=0`, `m=0`, min=max=half).
    fn from(half: i32) -> Self {
        let half = half.max(1);
        return Self {
            k: 0.0,
            pad: half,
            m: 0.0,
            half_min: half,
            half_max: half,
        };
    }
}

impl RoiParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.k >= 0.0, "roi.k >= 0");
        ensure!(self.m >= 0.0, "roi.m >= 0");
        ensure!(self.pad >= 0, "roi.pad >= 0");
        ensure!(self.half_min >= 1, "roi.half_min >= 1");
        ensure!(
            self.half_max >= self.half_min,
            "roi.half_max >= half_min"
        );
        return Ok(());
    }

    /// `area` 없으면 r=0. `delta_px`는 픽셀 이동 거리.
    pub fn compute_half(&self, area: Option<f64>, delta_px: f64) -> i32 {
        let r = area
            .filter(|a| a.is_finite() && *a > 0.0)
            .map(|a| (a / std::f64::consts::PI).sqrt())
            .unwrap_or(0.0);
        let delta = delta_px.max(0.0);
        let raw = self.k * r + f64::from(self.pad) + self.m * delta;
        return (raw.round() as i32).clamp(self.half_min, self.half_max);
    }

    /// `defaults::vision` paste용.
    pub fn to_defaults_snippet(&self) -> String {
        return format!(
            "RoiParams {{\n    k: {:.2},\n    pad: {},\n    m: {:.2},\n    half_min: {},\n    half_max: {},\n}}",
            self.k, self.pad, self.m, self.half_min, self.half_max
        );
    }
}

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

    #[test]
    fn roi_fixed_from_i32() {
        let p = RoiParams::from(80);
        assert_eq!(p.compute_half(Some(10_000.0), 50.0), 80);
        assert!(p.validate().is_ok());
    }

    #[test]
    fn roi_grows_with_area_and_delta() {
        let p = RoiParams {
            k: 3.0,
            pad: 10,
            m: 1.0,
            half_min: 20,
            half_max: 200,
        };
        // r=10 → 3*10+10 = 40
        assert_eq!(p.compute_half(Some(std::f64::consts::PI * 100.0), 0.0), 40);
        // +Δ20 → 60
        assert_eq!(p.compute_half(Some(std::f64::consts::PI * 100.0), 20.0), 60);
    }
}
