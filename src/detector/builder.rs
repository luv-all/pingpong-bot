//! 본선 검출 번들 + SimScene형 빌더.
//!
//! ```ignore
//! let color = ColormaskDetector::new(params);
//! let edges = ContourDetector::from(&scorer);
//! Detector::builder()
//!     .mask(FloorEdgeMask::from_params(cam_id, &cam)?)
//!     .then(color)
//!     .then(edges)
//!     .scorer(Scorer::from(&scorer).with_motion_weight(0.5))
//!     .roi(RoiParams::default())
//!     .build()?;
//! ```

use anyhow::{Result, bail};

use crate::PixelPoint;
use crate::camera::Frame;
use crate::detector::spatial::FloorEdgeMask;
use crate::detector::{
    AppearanceChain, AppearanceLayer, RoiParams, RoiTrack, Scorer, ScorerParams, track,
};

/// 조립된 본선 검출기 — mask / roi / scorer 스냅샷.
pub struct Detector {
    pub mask: FloorEdgeMask,
    pub roi: RoiTrack,
    /// 면적 밴드 HUD용 스냅샷.
    pub scorer: ScorerParams,
}

impl Detector {
    pub fn builder() -> DetectorBuilder {
        return DetectorBuilder::default();
    }

    /// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
    pub fn passthrough(hint: Option<PixelPoint>) -> Option<PixelPoint> {
        return super::passthrough_detect(hint);
    }

    /// 렌즈 왜곡 보정. 실패 시 에러 문자열.
    pub fn undistort(frame: &Frame, params: &crate::camera::CameraParams) -> Result<Frame, String> {
        return super::undistort_frame(frame, params);
    }

    pub fn set_roi_enabled(&mut self, enabled: bool) {
        self.roi.set_roi_enabled(enabled);
    }

    pub fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        let Ok(masked) = self.mask.apply_bgr(&frame.image) else {
            return None;
        };
        let gated = Frame {
            camera_id: frame.camera_id,
            image: masked,
            timestamp: frame.timestamp,
        };
        return self.roi.detect(&gated);
    }

    pub fn last_area(&self) -> Option<f64> {
        return self.roi.last_area();
    }
}

impl std::fmt::Display for Detector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(f, "detector({})", self.roi);
    }
}

/// [`Detector`] 조립. mask는 전처리 고정, `.then` 순서만 appearance 게이트 순서.
#[derive(Default)]
pub struct DetectorBuilder {
    mask: Option<FloorEdgeMask>,
    appearance: AppearanceChain,
    fuse_scorer: Option<Scorer>,
    scorer_params: Option<ScorerParams>,
    roi: Option<RoiParams>,
}

impl DetectorBuilder {
    pub fn mask(mut self, mask: FloorEdgeMask) -> Self {
        self.mask = Some(mask);
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
