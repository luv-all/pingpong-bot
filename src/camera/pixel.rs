//! 이미지 픽셀 좌표.

/// 이미지 픽셀 좌표.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pixel {
    pub x: f64,
    pub y: f64,
}

impl Pixel {
    pub fn new(x: f64, y: f64) -> Self {
        return Self { x, y };
    }

    pub fn lerp(self, other: Self, w: f64) -> Self {
        return Self {
            x: self.x + (other.x - self.x) * w,
            y: self.y + (other.y - self.y) * w,
        };
    }
}
