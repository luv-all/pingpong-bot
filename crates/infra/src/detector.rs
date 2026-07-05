//! 공 검출 어댑터 (1단계: passthrough).

use pingpong_domain::{Detector, FrameRef, PixelPoint, Roi};

/// 프레임 픽셀을 그대로 반환하는 스텁 검출기.
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
