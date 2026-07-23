//! 공 검출 — `BallDetector` 구현 + sim 패스스루.
//!
//! - [`appearance`] — colormask / contour / cascade
//! - [`fuse_layer`] — candidate · scorer · fuse
//! - [`motion`] — `MotionPrior`
//!
//! **조립 SSOT:** [`crate::defaults::detector`].

pub mod appearance;
pub mod fuse_layer;
pub mod motion;
mod track;
mod undistort;

use crate::PixelPoint;
use crate::camera::Frame;

pub use appearance::*;
pub use fuse_layer::candidate::{self as candidate, Candidate};
pub use fuse_layer::fuse::{
    self as fuse, CandidateGenerator, FuseDetector, IntoCandidateGenerators, fuse,
};
pub use fuse_layer::params::{self as params, RoiParams, ScorerParams};
pub use fuse_layer::scorer::{self as scorer, Scorer};
pub use motion::MotionPrior;
pub use track::{RoiTrack, track};
pub use undistort::undistort_frame;

/// 프레임에서 공 픽셀을 찾는다. `detect_*` 툴과 런타임이 공유한다.
pub trait BallDetector: Send {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint>;

    fn last_area(&self) -> Option<f64> {
        return None;
    }

    fn last_generator_idx(&self) -> Option<usize> {
        return None;
    }
}

/// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
pub fn passthrough_detect(hint: Option<PixelPoint>) -> Option<PixelPoint> {
    return hint;
}
