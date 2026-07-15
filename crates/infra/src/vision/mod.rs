//! 비전 SSOT — 캘리브·삼각측량·검출.
//!
//! OpenCV feature가 꺼져 있으면 nalgebra DLT 폴백으로 sim/CI가 돈다.
//! `opencv` feature를 켜면 `triangulatePoints` 등 OpenCV 경로를 탄다.

mod calib;
mod capture;
mod detect;
mod triangulate;

#[cfg(feature = "opencv")]
mod charuco;
#[cfg(feature = "opencv")]
mod opencv_tri;

pub use calib::{Calibration, CameraParams};
pub use capture::FrameSource;
pub use detect::{PassthroughDetector, passthrough_detect};
pub use triangulate::{dlt_triangulate, sample_at, triangulate_projections, triangulate_synced};

#[cfg(feature = "opencv")]
pub use charuco::calibrate_charuco_draft;
