//! 공 검출 어댑터.
//!
//! sim 은 카메라가 이미 픽셀을 넘기므로 passthrough.
//! 실물 OpenCV 는 tools/detect_* 실험 후 여기로 이식.

use pingpong_domain::{Detector, PixelPoint};

/// hint 픽셀을 그대로 반환 (sim/합성 카메라용).
pub struct PassthroughDetector;

impl PassthroughDetector {
    pub fn new() -> Self {
        return Self;
    }
}

impl Default for PassthroughDetector {
    fn default() -> Self {
        return Self::new();
    }
}

impl Detector for PassthroughDetector {
    fn detect(&mut self, hint: Option<PixelPoint>) -> Option<PixelPoint> {
        return hint;
    }
}
