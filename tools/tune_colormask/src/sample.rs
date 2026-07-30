//! 픽한 픽셀 샘플.

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub x: i32,
    pub y: i32,
    pub bgr: [u8; 3],
}
