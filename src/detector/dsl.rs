//! 검출 조립 DSL — 툴·런타임 공통 SSOT.
//!
//! ```ignore
//! // 단일 appearance
//! fuse(ColormaskDetector::try_from(&colormask)?, Scorer::from(&scorer))
//!     .with_motion_weight(w);
//!
//! // 여러 appearance (FirstSurviving)
//! fuse(
//!     generators![colormask, contour],
//!     Scorer::from(&scorer).with_motion_weight(w),
//! )
//! .with_motion_weight(w);
//!
//! // ROI
//! track(det, roi_half_px);
//! ```
//!
//! TOML → 위 DSL: [`fuse_vision`] / [`track_vision`].

use anyhow::{Result, ensure};

use super::appearance::{ColormaskDetector, ContourDetector};
use super::fuse::{CandidateGenerator, FuseDetector, fuse};
use super::params::{Appearance, VisionConfig};
use super::scorer::Scorer;
use super::track::{RoiTrack, track};
use crate::generators;

/// [`Scorer::from`] + colormask area 교집합 + motion weight.
pub fn scorer_from_vision(vision: &VisionConfig) -> Scorer {
    let mut scorer = Scorer::from(&vision.scorer).with_motion_weight(vision.motion.weight);
    if vision.generators.contains(&Appearance::Colormask) {
        scorer.min_area_px = scorer
            .min_area_px
            .max(vision.appearance.colormask.min_area_px);
        scorer.max_area_px = scorer
            .max_area_px
            .min(vision.appearance.colormask.max_area_px);
    }
    return scorer;
}

/// TOML `[vision]` → `fuse(…).with_motion_weight(w)`.
pub fn fuse_vision(vision: &VisionConfig) -> Result<FuseDetector> {
    ensure!(
        !vision.generators.is_empty(),
        "vision.generators는 비어 있으면 안 됩니다"
    );

    let w = vision.motion.weight;
    let scorer = scorer_from_vision(vision);

    let det = match vision.generators.as_slice() {
        [Appearance::Colormask] => fuse(
            ColormaskDetector::try_from(&vision.appearance.colormask)?,
            scorer,
        )
        .with_motion_weight(w),

        [Appearance::Contour] => {
            fuse(ContourDetector::from(&vision.scorer), scorer).with_motion_weight(w)
        }

        [Appearance::Colormask, Appearance::Contour] => fuse(
            generators![
                ColormaskDetector::try_from(&vision.appearance.colormask)?,
                ContourDetector::from(&vision.scorer),
            ],
            scorer,
        )
        .with_motion_weight(w),

        [Appearance::Contour, Appearance::Colormask] => fuse(
            generators![
                ContourDetector::from(&vision.scorer),
                ColormaskDetector::try_from(&vision.appearance.colormask)?,
            ],
            scorer,
        )
        .with_motion_weight(w),

        // 임의 길이·중복 — generators! 로 못 펼 때
        gens => {
            let mut boxes: Vec<Box<dyn CandidateGenerator>> = Vec::with_capacity(gens.len());
            for appearance in gens {
                match appearance {
                    Appearance::Colormask => {
                        boxes.push(Box::new(ColormaskDetector::try_from(
                            &vision.appearance.colormask,
                        )?));
                    }
                    Appearance::Contour => {
                        boxes.push(Box::new(ContourDetector::from(&vision.scorer)));
                    }
                }
            }
            fuse(boxes, scorer).with_motion_weight(w)
        }
    };

    return Ok(det);
}

/// `track(fuse_vision(vision)?, vision.roi_half_px)`.
pub fn track_vision(vision: &VisionConfig) -> Result<RoiTrack> {
    return Ok(track(fuse_vision(vision)?, vision.roi_half_px));
}
