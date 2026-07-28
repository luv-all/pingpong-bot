//! 공 검출 — [`Detector`] 본선.
//!
//! - [`appearance`] — colormask / contour / `.then` 체인
//! - [`scoring`] — candidate · scorer
//! - [`motion`] — `MotionPrior`
//! - [`spatial`] — floor mask · 면적 밴드
//! - [`builder`] — [`Detector`] 조립
//!
//! **조립 SSOT:** [`crate::defaults::detector_for`].

pub mod appearance;
pub mod builder;
pub mod motion;
pub mod scoring;
pub mod spatial;
mod roi_params;
mod track;
mod undistort;

use crate::PixelPoint;

pub use appearance::*;
pub use builder::{Detector, DetectorBuilder};
pub use motion::MotionPrior;
pub use roi_params::RoiParams;
pub use scoring::candidate::{self as candidate, Candidate};
pub use scoring::params::ScorerParams;
pub use scoring::scorer::{self as scorer, Scorer};
pub use spatial::{FloorEdgeMask, scorer_params_from_calib};
pub use track::RoiTrack;
pub use undistort::undistort_frame;

pub(crate) use track::track;

/// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
pub fn passthrough_detect(hint: Option<PixelPoint>) -> Option<PixelPoint> {
    return hint;
}
