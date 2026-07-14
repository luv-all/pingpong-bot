//! 공 검출.

use pingpong_domain::PixelPoint;

/// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
pub fn passthrough_detect(hint: Option<PixelPoint>) -> Option<PixelPoint> {
    return hint;
}

/// hint 패스스루 검출기 (상태 없음).
#[derive(Debug, Default, Clone, Copy)]
pub struct PassthroughDetector;

impl PassthroughDetector {
    pub fn new() -> Self {
        return Self;
    }

    pub fn detect(&mut self, hint: Option<PixelPoint>) -> Option<PixelPoint> {
        return passthrough_detect(hint);
    }
}
