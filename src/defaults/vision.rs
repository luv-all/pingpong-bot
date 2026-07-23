//! 공 검출 조립 (`fuse` · `track`).

use crate::detector::{
    ColorContourCascade, ColorSpace, ColormaskParams, RoiParams, RoiTrack, Scorer, ScorerParams,
    fuse, track,
};

const MOTION_WEIGHT: f64 = 0.5;

pub fn scorer() -> ScorerParams {
    return ScorerParams {
        min_area_px: 20.0,
        max_area_px: 20_000.0,
        min_circularity: 0.55,
    };
}

pub fn colormask() -> ColormaskParams {
    // paste into defaults::colormask() — space=ycrcb (Y/Cr/Cb)
    ColormaskParams {
        space: ColorSpace::Ycrcb,
        c0_min: 172, // Y
        c0_max: 250,
        c1_min: 131, // Cr
        c1_max: 188,
        c2_min: 7, // Cb
        c2_max: 94,
    }

    // // paste into defaults::colormask() — space=hsv (H/S/V)
    // ColormaskParams {
    //     space: ColorSpace::Hsv,
    //     c0_min: 15, // H
    //     c0_max: 33,
    //     c1_min: 71, // S
    //     c1_max: 245,
    //     c2_min: 252, // V
    //     c2_max: 255,
    // }
}

/// Adaptive ROI — detect-full에서 튜닝 후 paste.
pub fn roi() -> RoiParams {
    return RoiParams {
        k: 3.5,
        pad: 24,
        m: 1.0,
        half_min: 48,
        half_max: 240,
    };
}

/// 본선: **colormask → contour** cascade + ROI track.
///
/// 색으로 줄인 뒤, 그 영역에서만 Canny. (`ColorContourCascade`)
pub fn detector() -> RoiTrack {
    let scorer = scorer();
    let cascade = ColorContourCascade::new(colormask(), &scorer);
    let fuse_det = fuse(
        cascade,
        Scorer::from(&scorer).with_motion_weight(MOTION_WEIGHT),
    )
    .with_motion_weight(MOTION_WEIGHT);
    return track(fuse_det, roi());
}
