//! Adaptive ROI 파라미터. 앱 [`Default`]는 [`crate::defaults::vision`].

use anyhow::{Result, ensure};

/// Adaptive ROI:
/// `half = clamp(radius_scale·√(area/π) + padding + motion_scale·Δ, half_min, half_max)`.
#[derive(Debug, Clone, PartialEq)]
pub struct RoiParams {
    /// 등가 반경 배율.
    pub radius_scale: f64,
    /// 고정 여유 [px].
    pub padding: i32,
    /// 직전 프레임 이동량 `|Δpx|` 배율.
    pub motion_scale: f64,
    pub half_min: i32,
    pub half_max: i32,
}

impl From<i32> for RoiParams {
    /// 고정 half — adaptive 끔 (`radius_scale=0`, `motion_scale=0`, min=max=half).
    fn from(half: i32) -> Self {
        let half = half.max(1);
        return Self {
            radius_scale: 0.0,
            padding: half,
            motion_scale: 0.0,
            half_min: half,
            half_max: half,
        };
    }
}

impl RoiParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.radius_scale >= 0.0, "roi.radius_scale >= 0");
        ensure!(self.motion_scale >= 0.0, "roi.motion_scale >= 0");
        ensure!(self.padding >= 0, "roi.padding >= 0");
        ensure!(self.half_min >= 1, "roi.half_min >= 1");
        ensure!(self.half_max >= self.half_min, "roi.half_max >= half_min");
        return Ok(());
    }

    /// `area` 없으면 r=0. `delta_px`는 픽셀 이동 거리.
    pub fn compute_half(&self, area: Option<f64>, delta_px: f64) -> i32 {
        let r = area
            .filter(|a| a.is_finite() && *a > 0.0)
            .map(|a| (a / std::f64::consts::PI).sqrt())
            .unwrap_or(0.0);
        let delta = delta_px.max(0.0);
        let raw = self.radius_scale * r + f64::from(self.padding) + self.motion_scale * delta;
        return (raw.round() as i32).clamp(self.half_min, self.half_max);
    }

    /// `defaults::vision` paste용.
    pub fn to_defaults_snippet(&self) -> String {
        return format!(
            "RoiParams {{\n    radius_scale: {:.2},\n    padding: {},\n    motion_scale: {:.2},\n    half_min: {},\n    half_max: {},\n}}",
            self.radius_scale, self.padding, self.motion_scale, self.half_min, self.half_max
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roi_fixed_from_i32() {
        let p = RoiParams::from(80);
        assert_eq!(p.compute_half(Some(10_000.0), 50.0), 80);
        assert!(p.validate().is_ok());
    }

    #[test]
    fn roi_grows_with_area_and_delta() {
        let p = RoiParams {
            radius_scale: 3.0,
            padding: 10,
            motion_scale: 1.0,
            half_min: 20,
            half_max: 200,
        };
        assert_eq!(p.compute_half(Some(std::f64::consts::PI * 100.0), 0.0), 40);
        assert_eq!(p.compute_half(Some(std::f64::consts::PI * 100.0), 20.0), 60);
    }
}
