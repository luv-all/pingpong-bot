//! 공 검출 어댑터.
//!
//! Phase 2: sim은 카메라가 이미 투영 픽셀을 넣으므로 passthrough.
//! 실물 OpenCV(HSV/contour)는 `tools/detect_*` 실험 후 여기로 이식.

use pingpong_domain::{Detector, FrameRef, PixelPoint, Roi};

/// 프레임에 실린 픽셀을 그대로 반환 (sim·합성 카메라용).
pub struct PassthroughDetector;

impl PassthroughDetector {
    /// 인스턴스를 만든다.
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
    fn detect(
        &mut self,
        frame: FrameRef,
        _roi: Option<Roi>,
    ) -> Option<PixelPoint> {
        return frame.pixel();
    }
}
