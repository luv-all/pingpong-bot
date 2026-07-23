//! 공 검출 조립 (`fuse` · `track`).

use crate::detector::{
    ColorSpace, ColormaskDetector, ColormaskParams, ContourDetector, RoiTrack, Scorer, ScorerParams,
    fuse, track,
};
use crate::generators;

const ROI_HALF_PX: i32 = 80;
const MOTION_WEIGHT: f64 = 0.5;

pub fn scorer() -> ScorerParams {
    return ScorerParams {
        min_area_px: 20.0,
        max_area_px: 20_000.0,
        min_circularity: 0.55,
    };
}

pub fn colormask() -> ColormaskParams {
    return ColormaskParams {
        space: ColorSpace::Ycrcb,
        c0_min: 0,
        c0_max: 255,
        c1_min: 133,
        c1_max: 173,
        c2_min: 77,
        c2_max: 127,
    };
}

/// `fuse(generators![…], scorer)` + ROI track.
pub fn detector() -> RoiTrack {
    let scorer = scorer();
    let fuse_det = fuse(
        generators![
            ColormaskDetector::new(colormask()),
            ContourDetector::from(&scorer),
        ],
        Scorer::from(&scorer).with_motion_weight(MOTION_WEIGHT),
    )
    .with_motion_weight(MOTION_WEIGHT);
    return track(fuse_det, ROI_HALF_PX);
}
