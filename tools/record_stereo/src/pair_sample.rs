//! 링 버퍼 JPEG 페어.

use std::time::Instant;

#[derive(Clone)]
pub struct PairSample {
    pub t: Instant,
    pub left_jpeg: Vec<u8>,
    pub right_jpeg: Vec<u8>,
    pub width: i32,
    pub height: i32,
}
