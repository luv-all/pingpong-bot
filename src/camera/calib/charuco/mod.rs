//! OpenCV ChArUco 보드 → 카메라 인트린식·왜곡.

mod board_spec;
mod calibrate;
mod detect;
mod frame_detect;
mod report;

pub use board_spec::BoardSpec;
pub use calibrate::calibrate_charuco;
pub use detect::detect_and_draw_charuco;
pub use frame_detect::FrameDetect;
pub use report::Report;

/// 프레임당 최소 ChArUco 코너 (저장·보정 후보).
pub use crate::defaults::calib::MIN_CHARUCO_CORNERS;
