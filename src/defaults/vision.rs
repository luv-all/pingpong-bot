//! 공 검출 조립 + 비전 UI — Params [`Default`]가 앱 프리셋.

use crate::detector::{
    ColorContourCascade, ColorSpace, ColormaskParams, RoiParams, RoiTrack, Scorer, ScorerParams,
    fuse, track,
};

/// fuse scorer motion 가중.
pub const MOTION_WEIGHT: f64 = 0.5;
/// MotionPrior absdiff 이진화 임계.
pub const MOTION_DIFF_THRESH: f64 = 25.0;
/// 픽셀 정밀 찍기용 loupe 배율.
pub const PIXEL_LOUPE_ZOOM: i32 = 8;
/// loupe 소스 반경 [px].
pub const PIXEL_LOUPE_SRC_HALF: i32 = 7;

impl Default for ScorerParams {
    fn default() -> Self {
        return Self {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.55,
        };
    }
}

impl Default for ColormaskParams {
    fn default() -> Self {
        // paste into defaults — space=ycrcb (Y/Cr/Cb)
        return Self {
            space: ColorSpace::Ycrcb,
            c0_min: 172, // Y
            c0_max: 250,
            c1_min: 131, // Cr
            c1_max: 188,
            c2_min: 7, // Cb
            c2_max: 94,
        };
    }
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

/// 본선: colormask → contour cascade + ROI track.
pub fn detector() -> RoiTrack {
    let scorer = ScorerParams::default();
    let cascade = ColorContourCascade::new(ColormaskParams::default(), &scorer);
    let fuse_det = fuse(
        cascade,
        Scorer::from(&scorer).with_motion_weight(MOTION_WEIGHT),
    )
    .with_motion_weight(MOTION_WEIGHT);
    return track(fuse_det, RoiParams::default());
}
