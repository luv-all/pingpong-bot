//! [`Detector`] 조립. mask는 전처리 고정, `.then` 순서만 appearance 게이트 순서.
//!
//! ```ignore
//! let color = ColormaskDetector::new(params);
//! let edges = ContourDetector::from(&scorer);
//! let floor = FloorEdgeMask::from_params(cam_id, &cam)?;
//! let corridor = TableCorridorMask::from_params(&cam, FLIGHT_BAND_M)?;
//! Detector::builder()
//!     .mask(SpatialMask::with_corridor(floor, corridor)?)
//!     .then(color)
//!     .then(edges)
//!     .scorer(Scorer::from(&scorer).with_motion_weight(0.5))
//!     .roi(RoiParams::default())
//!     .build()?;
//! ```

use anyhow::{Result, bail};

use crate::detector::spatial::SpatialMask;
use crate::detector::{AppearanceChain, AppearanceLayer, RoiParams, Scorer, ScorerParams, track};

use super::detector::Detector;

#[derive(Default)]
pub struct DetectorBuilder {
    mask: Option<SpatialMask>,
    appearance: AppearanceChain,
    fuse_scorer: Option<Scorer>,
    scorer_params: Option<ScorerParams>,
    roi: Option<RoiParams>,
}

impl DetectorBuilder {
    /// 공간 keep. [`FloorEdgeMask`] 하나만 줘도 `From`으로 받는다.
    pub fn mask(mut self, mask: impl Into<SpatialMask>) -> Self {
        self.mask = Some(mask.into());
        return self;
    }

    /// appearance 레이어 추가. 호출 순서 = 게이트 순서.
    pub fn then(mut self, layer: impl AppearanceLayer + 'static) -> Self {
        self.appearance.push(layer);
        return self;
    }

    pub fn scorer(mut self, scorer: Scorer) -> Self {
        self.scorer_params = Some(ScorerParams {
            min_area_px: scorer.min_area_px,
            max_area_px: scorer.max_area_px,
            min_circularity: scorer.min_circularity,
        });
        self.fuse_scorer = Some(scorer);
        return self;
    }

    pub fn roi(mut self, params: impl Into<RoiParams>) -> Self {
        self.roi = Some(params.into());
        return self;
    }

    pub fn build(self) -> Result<Detector> {
        let Some(mask) = self.mask else {
            bail!("Detector::builder: .mask(...) required");
        };
        if self.appearance.is_empty() {
            bail!("Detector::builder: at least one .then(...) appearance layer");
        }
        let Some(fuse_scorer) = self.fuse_scorer else {
            bail!("Detector::builder: .scorer(...) required");
        };
        let scorer_params = self.scorer_params.unwrap_or_else(|| ScorerParams {
            min_area_px: fuse_scorer.min_area_px,
            max_area_px: fuse_scorer.max_area_px,
            min_circularity: fuse_scorer.min_circularity,
        });
        let roi_params = self.roi.unwrap_or_default();
        let roi = track(self.appearance, fuse_scorer, roi_params);
        return Ok(Detector {
            mask,
            roi,
            scorer: scorer_params,
        });
    }
}
