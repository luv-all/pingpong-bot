//! Appearance 레이어 — 색/엣지 기반 [`super::fuse::CandidateGenerator`] 구현.

mod cascade;
mod colormask;
mod contour;

pub use cascade::ColorContourCascade;
pub use colormask::{
    ColorSpace, ColormaskBgr, ColormaskCam, ColormaskDetector, ColormaskParams, ColormaskSet,
    ParseColorSpaceError, load_colormask_set, load_colormask_set_or_empty, save_colormask_set,
};
pub use contour::ContourDetector;
