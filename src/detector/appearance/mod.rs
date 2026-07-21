//! Appearance 레이어 — 색/엣지 기반 [`super::fuse::CandidateGenerator`] 구현.

mod colormask;
mod contour;

pub use colormask::{ColorSpace, ColormaskConfig, ColormaskDetector, ParseColorSpaceError};
pub use contour::ContourDetector;
