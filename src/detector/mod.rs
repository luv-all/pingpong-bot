//! 공 검출 — `BallDetector` 구현 + sim 패스스루.
//!
//! 모듈 = fuse 레이어 그대로 nesting:
//! 1. [`appearance`] — [`CandidateGenerator`] 구현체: `colormask` / `contour`
//! 2. [`scorer`] — area · circularity · motion soft weight
//! 3. [`motion`] — `MotionPrior` 마스크 (optional soft boost)
//!
//! **조립 DSL SSOT:** [`dsl`] (`fuse_vision` / `track_vision`).
//!
//! ```ignore
//! use pingpong_bot::{fuse, generators, track, ColormaskDetector, Scorer};
//!
//! let det = fuse(
//!     ColormaskDetector::new(cfg),
//!     Scorer::shape(20.0, 20_000.0, 0.55),
//! )
//! .with_motion_weight(0.5);
//!
//! let det = fuse(
//!     generators![colormask, contour],
//!     Scorer::shape(20.0, 20_000.0, 0.55).with_motion_weight(0.5),
//! )
//! .with_motion_weight(0.5);
//!
//! let mut d = track(det, 80);
//! // TOML:
//! let mut d = track_vision(&vision)?;
//! ```

mod appearance;
mod candidate;
pub mod dsl;
mod fuse;
mod motion;
mod params;
mod scorer;
mod track;
mod undistort;

use crate::PixelPoint;
use crate::camera::Frame;

// DX: appearance는 `detector::appearance::{...}`로도, detector 루트로도 쓸 수 있다.
pub use appearance::*;
pub use candidate::Candidate;
pub use dsl::{fuse_vision, scorer_from_vision, track_vision};
pub use fuse::{CandidateGenerator, FuseDetector, IntoCandidateGenerators, fuse};
pub use motion::MotionPrior;
pub use params::{
    Appearance, AppearanceParams, ColormaskParams, MotionParams, ParseAppearanceError,
    ScorerParams, VisionCameraConfig, VisionConfig, fuse_from_vision, load_vision_from_config,
    vision_from_toml,
};
pub use scorer::Scorer;
pub use track::{RoiTrack, track};
pub use undistort::undistort_frame;

/// 프레임에서 공 픽셀을 찾는다. `detect_*` 툴과 런타임이 공유한다.
pub trait BallDetector: Send {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint>;
}

/// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
pub fn passthrough_detect(hint: Option<PixelPoint>) -> Option<PixelPoint> {
    return hint;
}
