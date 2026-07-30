//! 한 프레임에서 검출한 공 관측.

use std::time::Instant;

use crate::camera;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub pixel: camera::Pixel,
    pub camera_id: camera::Id,
    pub timestamp: Instant,
}
