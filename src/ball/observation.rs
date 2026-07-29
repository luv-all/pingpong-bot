//! 한 프레임에서 검출한 공 관측.

use std::time::Instant;

use crate::camera::{CameraId, PixelPoint};

/// 한 프레임에서 검출한 공.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub pixel: PixelPoint,
    pub camera_id: CameraId,
    pub timestamp: Instant,
}
