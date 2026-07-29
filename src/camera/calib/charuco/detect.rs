//! BGR 프레임 ChArUco 검출·오버레이.

use opencv::core::Mat;
use opencv::core::{Point2f, Size, Vector};
use opencv::imgproc;
use opencv::objdetect::{
    self, CharucoBoard, CharucoDetector, CharucoParameters, DetectorParameters,
    PredefinedDictionaryType, RefineParameters, get_predefined_dictionary,
};
use opencv::prelude::*;

use super::{BoardSpec, FrameDetect, MIN_CHARUCO_CORNERS};

fn make_charuco_detector(board_spec: BoardSpec) -> Result<(CharucoBoard, CharucoDetector), String> {
    let dict = get_predefined_dictionary(PredefinedDictionaryType::DICT_4X4_50)
        .map_err(|e| format!("dictionary: {e}"))?;
    let board = CharucoBoard::new_def(
        Size::new(board_spec.squares_x, board_spec.squares_y),
        board_spec.square_length_m,
        board_spec.marker_length_m,
        &dict,
    )
    .map_err(|e| format!("board: {e}"))?;
    let charuco_params =
        CharucoParameters::default().map_err(|e| format!("charuco_params: {e}"))?;
    let detector_params =
        DetectorParameters::default().map_err(|e| format!("detector_params: {e}"))?;
    let refine_params = RefineParameters::new_def().map_err(|e| format!("refine_params: {e}"))?;
    let detector = CharucoDetector::new(&board, &charuco_params, &detector_params, refine_params)
        .map_err(|e| format!("detector: {e}"))?;
    return Ok((board, detector));
}

pub fn detect_and_draw_charuco(
    bgr: &Mat,
    board_spec: BoardSpec,
) -> Result<(Mat, FrameDetect), String> {
    let (board, detector) = make_charuco_detector(board_spec)?;
    let mut gray = Mat::default();
    imgproc::cvt_color(
        bgr,
        &mut gray,
        imgproc::COLOR_BGR2GRAY,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )
    .map_err(|e| format!("cvt_color: {e}"))?;

    let mut charuco_corners = Vector::<Point2f>::new();
    let mut charuco_ids = Vector::<i32>::new();
    let mut marker_corners = Vector::<Vector<Point2f>>::new();
    let mut marker_ids = Vector::<i32>::new();
    detector
        .detect_board(
            &gray,
            &mut charuco_corners,
            &mut charuco_ids,
            &mut marker_corners,
            &mut marker_ids,
        )
        .map_err(|e| format!("detect_board: {e}"))?;
    let _board_alive = board;

    let mut overlay = bgr.try_clone().map_err(|e| format!("clone: {e}"))?;
    if !marker_corners.is_empty() {
        objdetect::draw_detected_markers(
            &mut overlay,
            &marker_corners,
            &marker_ids,
            opencv::core::Scalar::new(0.0, 255.0, 0.0, 0.0),
        )
        .map_err(|e| format!("draw_markers: {e}"))?;
    }
    if !charuco_corners.is_empty() {
        objdetect::draw_detected_corners_charuco(
            &mut overlay,
            &charuco_corners,
            &charuco_ids,
            opencv::core::Scalar::new(255.0, 0.0, 255.0, 0.0),
        )
        .map_err(|e| format!("draw_charuco: {e}"))?;
    }

    let corners = charuco_ids.len();
    let markers = marker_ids.len();
    let ok = corners >= MIN_CHARUCO_CORNERS;
    return Ok((
        overlay,
        FrameDetect {
            corners,
            markers,
            ok,
        },
    ));
}
