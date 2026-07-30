//! 공 검출 — [`Detector`] 본선.
//!
//! - [`appearance`] — colormask / contour / `.then` 체인
//! - [`scoring`] — candidate · scorer
//! - [`motion`] — `MotionPrior`
//! - [`spatial`] — floor mask · 테이블 복도 · 면적 밴드
//! - [`builder`] — [`Detector`] 조립
//!
//! **조립 SSOT:** [`crate::defaults::detector_for`].

pub mod appearance;
mod builder;
mod detector;
pub mod motion;
mod observation;
mod roi_params;
pub mod scoring;
pub mod spatial;
mod track;
mod undistort;

use crate::camera;

pub use appearance::*;
pub use builder::DetectorBuilder;
pub use detector::Detector;
pub use motion::MotionPrior;
pub use observation::Observation;
pub use roi_params::RoiParams;
pub use scoring::candidate::{self as candidate, Candidate};
pub use scoring::params::ScorerParams;
pub use scoring::scorer::{self as scorer, Scorer};
pub(crate) use spatial::scorer_params_from_calib;
pub use spatial::{FloorEdgeMask, SpatialMask, TableCorridorMask};
pub use track::RoiTrack;
pub(crate) use undistort::undistort_frame;

pub(crate) use track::track;

/// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
pub(crate) fn passthrough_detect(hint: Option<camera::Pixel>) -> Option<camera::Pixel> {
    return hint;
}
