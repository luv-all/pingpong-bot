//! 카메라 입력·캘리브레이션·삼각측량.
//!
//! 세부 토픽:
//! - [`calib`] — `Calibration` / ChArUco / 탁구대 PnP
//! - [`tri`] — DLT · OpenCV `triangulatePoints`
//! - [`preview`] — highgui 오버레이
//! - capture / projection / sim — 입력·투영·시뮬 카메라

use std::fmt;
use std::time::Instant;

pub mod calib;
mod capture;
pub mod preview;
mod projection;
mod sim;
pub mod tri;

pub use calib::{
    Calibration, CameraParams, CharucoBoardSpec, CharucoCalibReport, CharucoFrameDetect,
    MAX_REPROJ_RMSE_PX, MIN_CHARUCO_CORNERS, TABLE_LANDMARK_COUNT, TableLandmark, TablePnpResult,
    calibrate_charuco, calibrate_table_pnp, detect_and_draw_charuco, ensure_reproj_below,
    ensure_reproj_ok, table_landmark_mesh_edges, table_landmarks, upsert_camera,
};
pub use capture::{
    ExposureReadout, Frame, FrameSource, HintSource, ImageDirSource, OpenCvCapture,
};
pub use preview::{
    PreviewAction, destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines,
    draw_help_lines, draw_world_velocity, hstack_bgr, show_bgr,
};
pub use sim::SimCamera;
pub use tri::{
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
