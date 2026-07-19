//! 카메라 입력·캘리브레이션·삼각측량.

use std::fmt;
use std::time::Instant;

mod calibration;
mod capture;
mod charuco;
mod opencv_tri;
pub mod preview;
mod projection;
mod sim;
mod synthetic;
mod triangulate;

pub use calibration::{Calibration, CameraParams};
pub use capture::{Frame, FrameSource, HintSource, ImageDirSource, OpenCvCapture};
pub use charuco::{
    CharucoBoardSpec, CharucoCalibReport, calibrate_charuco, calibrate_charuco_draft,
};
pub use preview::{PreviewAction, destroy_window, draw_debug_lines, hstack_bgr, show_bgr};
pub use sim::SimCamera;
pub use synthetic::SyntheticCamera;
pub use triangulate::{
    dlt_triangulate, sample_at, triangulate_projections, triangulate_synced, triangulate_views,
};

/// 이미지 픽셀 좌표.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelPoint {
    pub x: f64,
    pub y: f64,
}

impl PixelPoint {
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

/// 카메라 식별자.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub struct CameraId(pub u8);

impl CameraId {
    pub const fn new(index: u8) -> Self {
        return Self(index);
    }

    pub fn index(self) -> u8 {
        return self.0;
    }
}

impl fmt::Display for CameraId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "카메라 {}번", self.0);
    }
}

/// 한 프레임에서 검출한 공.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallObservation {
    pub pixel: PixelPoint,
    pub camera_id: CameraId,
    pub timestamp: Instant,
}
