//! 검출 전 프레임에 [`FloorEdgeMask`] 적용.

use super::FloorEdgeMask;
use crate::PixelPoint;
use crate::camera::Frame;
use crate::detector::{BallDetector, RoiTrack};

/// `detect` 진입 시 BGR에 바닥 마스크를 씌운 뒤 `inner`에 위임.
pub struct SpatialGate {
    pub mask: FloorEdgeMask,
    pub inner: RoiTrack,
}

impl SpatialGate {
    pub fn new(mask: FloorEdgeMask, inner: RoiTrack) -> Self {
        return Self { mask, inner };
    }

    pub fn set_roi_enabled(&mut self, enabled: bool) {
        self.inner.set_roi_enabled(enabled);
    }
}

impl std::ops::Deref for SpatialGate {
    type Target = RoiTrack;

    fn deref(&self) -> &RoiTrack {
        return &self.inner;
    }
}

impl std::ops::DerefMut for SpatialGate {
    fn deref_mut(&mut self) -> &mut RoiTrack {
        return &mut self.inner;
    }
}

impl std::fmt::Display for SpatialGate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(f, "spatial+{}", self.inner);
    }
}

impl BallDetector for SpatialGate {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        let Ok(masked) = self.mask.apply_bgr(&frame.image) else {
            return None;
        };
        let gated = Frame {
            camera_id: frame.camera_id,
            image: masked,
            timestamp: frame.timestamp,
        };
        return self.inner.detect(&gated);
    }

    fn last_area(&self) -> Option<f64> {
        return self.inner.last_area();
    }

    fn last_generator_idx(&self) -> Option<usize> {
        return self.inner.last_generator_idx();
    }
}
