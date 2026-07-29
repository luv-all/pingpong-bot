//! 한 프레임에서 검출한 공 관측.

use std::time::Instant;

use crate::camera::{Id, Pixel};

/// 한 프레임에서 검출한 공.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub pixel: Pixel,
    pub camera_id: Id,
    pub timestamp: Instant,
}
