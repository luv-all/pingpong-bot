//! Appearance 레이어 — 색/엣지 마스크 · `.then` 체인.

mod chain;
mod colormask;
mod contour;
mod generator;
mod layer;

pub use chain::AppearanceChain;
pub use colormask::{
    ColorSpace, ColormaskBgr, ColormaskCam, ColormaskDetector, ColormaskParams, ColormaskSet,
    ParseColorSpaceError, load_colormask_set, load_colormask_set_or_empty, save_colormask_set,
};
pub use contour::ContourDetector;
pub use generator::CandidateGenerator;
pub use layer::AppearanceLayer;
