//! 공 검출 — `BallDetector` 구현 + sim 패스스루.
//!
//! 모듈 = fuse 레이어 그대로 nesting:
//! 1. [`appearance`] — [`CandidateGenerator`] 구현체: `colormask` / `contour`
//! 2. [`scorer`] — area · circularity · motion soft weight
//! 3. [`motion`] — `MotionPrior` 마스크 (optional soft boost)
//!
//! **조립 SSOT:** [`crate::defaults::detector`].
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
//! let mut d = track(det, RoiParams::default());
//! // 또는 고정 half: track(det, 80)
//! let mut d = pingpong_bot::detector();
//! ```

mod appearance;
mod candidate;
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
pub use fuse::{CandidateGenerator, FuseDetector, IntoCandidateGenerators, fuse};
pub use motion::MotionPrior;
pub use params::{RoiParams, ScorerParams};
pub use scorer::Scorer;
pub use track::{RoiTrack, track};
pub use undistort::undistort_frame;

/// 프레임에서 공 픽셀을 찾는다. `detect_*` 툴과 런타임이 공유한다.
pub trait BallDetector: Send {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint>;

    /// 직전 hit 우승 candidate 면적 [px²]. ROI adaptive용 side-channel.
    fn last_area(&self) -> Option<f64> {
        return None;
    }

    /// FirstSurviving 우승 generator 인덱스 (`fuse`만).
    fn last_generator_idx(&self) -> Option<usize> {
        return None;
    }
}

/// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
pub fn passthrough_detect(hint: Option<PixelPoint>) -> Option<PixelPoint> {
    return hint;
}
