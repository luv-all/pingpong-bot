//! Appearance 레이어 — 색/엣지 기반 [`super::fuse::CandidateGenerator`] 구현.

mod cascade;
mod colormask;
mod contour;

pub use cascade::ColorContourCascade;
pub use colormask::{ColorSpace, ColormaskParams, ColormaskDetector, ParseColorSpaceError};
pub use contour::ContourDetector;
