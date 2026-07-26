//! Appearance 레이어 — 색/엣지 기반 [`super::fuse::CandidateGenerator`] 구현.

mod cascade;
mod colormask;
mod contour;

pub use cascade::ColorContourCascade;
pub use colormask::{ColorSpace, ColormaskDetector, ColormaskParams, ParseColorSpaceError};
pub use contour::ContourDetector;
