//! ChArUco 보정 공개 진입점.

use crate::camera;
use crate::camera::calib::{Calibration, FrameDetect, Report};

/// ChArUco 보정 공개 진입점.
pub struct Charuco;

impl Charuco {
    pub fn calibrate(
        dir: &std::path::Path,
        board_spec: camera::BoardSpec,
        camera_id: camera::Id,
    ) -> Result<(Calibration, Report), String> {
        return crate::camera::calib::calibrate_charuco(dir, board_spec, camera_id);
    }

    pub fn detect_and_draw(
        bgr: &opencv::core::Mat,
        board_spec: camera::BoardSpec,
    ) -> Result<(opencv::core::Mat, FrameDetect), String> {
        return crate::camera::calib::detect_and_draw_charuco(bgr, board_spec);
    }
}
