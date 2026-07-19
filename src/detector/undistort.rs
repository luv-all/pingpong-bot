//! Calibration `dist`로 프레임 undistort.

use opencv::calib3d;
use opencv::core::Mat;
use opencv::prelude::*;

use crate::camera::{CameraParams, Frame};

/// `dist`가 비어 있으면 원본 프레임을 그대로 돌려준다.
pub fn undistort_frame(frame: &Frame, params: &CameraParams) -> Result<Frame, String> {
    if !params.has_distortion() {
        return Ok(Frame::new(
            frame.camera_id,
            frame.image.try_clone().map_err(|e| format!("clone: {e}"))?,
            frame.timestamp,
        ));
    }

    let camera_matrix = Mat::from_slice_2d(&[
        &[params.fx, 0.0, params.cx],
        &[0.0, params.fy, params.cy],
        &[0.0, 0.0, 1.0],
    ])
    .map_err(|e| format!("K: {e}"))?;
    let dist = Mat::from_slice(&params.dist).map_err(|e| format!("dist: {e}"))?;

    let mut undistorted = Mat::default();
    calib3d::undistort(
        &frame.image,
        &mut undistorted,
        &camera_matrix,
        &dist,
        &opencv::core::no_array(),
    )
    .map_err(|e| format!("undistort: {e}"))?;

    return Ok(Frame::new(frame.camera_id, undistorted, frame.timestamp));
}
