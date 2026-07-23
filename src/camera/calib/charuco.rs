//! OpenCV ChArUco 보드 → 카메라 인트린식·왜곡 (`Calibration` JSON).
//!
//! 외부 R|t는 피팅하지 않는다. 외참은 [`super::table_pnp`] / `calib-table-pnp`를 쓴다.
//! 이 모듈은 K·dist만 덮어쓰고, sim look-at을 자리표시자로 둔다.

use std::ffi::OsStr;
use std::fs;
use std::path::Path;

use opencv::core::{Point2f, Point3f, Size, TermCriteria, TermCriteria_Type, Vector};
use opencv::objdetect::{
    self, CharucoBoard, CharucoDetector, CharucoParameters, DetectorParameters,
    PredefinedDictionaryType, RefineParameters, get_predefined_dictionary,
};
use opencv::prelude::*;
use opencv::{calib3d, imgcodecs, imgproc};

use super::Calibration;
use crate::CameraId;

/// ChArUco 보드 규격 (CLI에서 덮어쓸 수 있음).
#[derive(Debug, Clone, Copy)]
pub struct CharucoBoardSpec {
    pub squares_x: i32,
    pub squares_y: i32,
    pub square_length_m: f32,
    pub marker_length_m: f32,
}

impl Default for CharucoBoardSpec {
    fn default() -> Self {
        return Self {
            squares_x: 5,
            squares_y: 7,
            square_length_m: 0.04,
            marker_length_m: 0.02,
        };
    }
}

/// 보정 결과 메타 (로그용).
#[derive(Debug, Clone)]
pub struct CharucoCalibReport {
    pub rms: f64,
    pub frames_used: usize,
    pub frames_total: usize,
}

/// `dir`의 이미지에서 ChArUco를 모아 인트린식+dist를 피팅한다.
pub fn calibrate_charuco(
    dir: &Path,
    board_spec: CharucoBoardSpec,
    camera_id: CameraId,
) -> Result<(Calibration, CharucoCalibReport), String> {
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

    let mut entries: Vec<_> = fs::read_dir(dir)
        .map_err(|e| format!("read_dir: {e}"))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            matches!(
                p.extension().and_then(OsStr::to_str),
                Some("png" | "jpg" | "jpeg" | "bmp")
            )
        })
        .collect();
    entries.sort();
    if entries.is_empty() {
        return Err(format!("이미지 없음: {}", dir.display()));
    }

    let mut all_obj = Vector::<Vector<Point3f>>::new();
    let mut all_img = Vector::<Vector<Point2f>>::new();
    let mut image_size = Size::default();
    let mut frames_used = 0usize;

    for path in &entries {
        let Some(path_str) = path.to_str() else {
            continue;
        };
        let img = imgcodecs::imread(path_str, imgcodecs::IMREAD_COLOR)
            .map_err(|e| format!("imread {}: {e}", path.display()))?;
        if img.empty() {
            continue;
        }
        image_size = img.size().map_err(|e| format!("size: {e}"))?;
        let mut gray = opencv::core::Mat::default();
        imgproc::cvt_color(
            &img,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .map_err(|e| format!("cvt_color: {e}"))?;

        let mut charuco_corners = Vector::<Point2f>::new();
        let mut charuco_ids = Vector::<i32>::new();
        detector
            .detect_board_def(&gray, &mut charuco_corners, &mut charuco_ids)
            .map_err(|e| format!("detect_board {}: {e}", path.display()))?;
        if charuco_ids.len() < 4 {
            continue;
        }

        let mut obj_points = Vector::<Point3f>::new();
        let mut img_points = Vector::<Point2f>::new();
        board
            .match_image_points(
                &charuco_corners,
                &charuco_ids,
                &mut obj_points,
                &mut img_points,
            )
            .map_err(|e| format!("match_image_points {}: {e}", path.display()))?;
        if obj_points.len() < 4 {
            continue;
        }
        all_obj.push(obj_points);
        all_img.push(img_points);
        frames_used += 1;
    }

    if frames_used == 0 {
        return Err("ChArUco 코너가 검출된 프레임이 없음 (최소 4 corners/frame)".into());
    }
    if frames_used < 3 {
        return Err(format!(
            "보정에 프레임이 부족함: {frames_used} (권장 ≥3, 가능하면 ≥10)"
        ));
    }

    let mut camera_matrix = opencv::core::Mat::default();
    let mut dist_coeffs = opencv::core::Mat::default();
    let mut rvecs = Vector::<opencv::core::Mat>::new();
    let mut tvecs = Vector::<opencv::core::Mat>::new();
    let criteria = TermCriteria::new(
        TermCriteria_Type::COUNT as i32 + TermCriteria_Type::EPS as i32,
        100,
        1e-6,
    )
    .map_err(|e| format!("TermCriteria: {e}"))?;

    let rms = calib3d::calibrate_camera(
        &all_obj,
        &all_img,
        image_size,
        &mut camera_matrix,
        &mut dist_coeffs,
        &mut rvecs,
        &mut tvecs,
        0,
        criteria,
    )
    .map_err(|e| format!("calibrate_camera: {e}"))?;

    let (fx, fy, cx, cy) = read_camera_matrix(&camera_matrix)?;
    let dist = read_dist_coeffs(&dist_coeffs)?;

    let mut calib = Calibration::sim(1);
    let cam = &mut calib.cameras[0];
    cam.camera_id = camera_id;
    cam.width = image_size.width.max(1) as u32;
    cam.height = image_size.height.max(1) as u32;
    cam.fx = fx;
    cam.fy = fy;
    cam.cx = cx;
    cam.cy = cy;
    cam.dist = dist;
    cam.label = Some(format!(
        "charuco rms={rms:.4} frames={frames_used}/{}",
        entries.len()
    ));

    return Ok((
        calib,
        CharucoCalibReport {
            rms,
            frames_used,
            frames_total: entries.len(),
        },
    ));
}

fn read_camera_matrix(k: &opencv::core::Mat) -> Result<(f64, f64, f64, f64), String> {
    let fx = *k.at_2d::<f64>(0, 0).map_err(|e| format!("K(0,0): {e}"))?;
    let fy = *k.at_2d::<f64>(1, 1).map_err(|e| format!("K(1,1): {e}"))?;
    let cx = *k.at_2d::<f64>(0, 2).map_err(|e| format!("K(0,2): {e}"))?;
    let cy = *k.at_2d::<f64>(1, 2).map_err(|e| format!("K(1,2): {e}"))?;
    return Ok((fx, fy, cx, cy));
}

fn read_dist_coeffs(d: &opencv::core::Mat) -> Result<Vec<f64>, String> {
    let total = d.total() as usize;
    let mut out = Vec::with_capacity(total);
    for i in 0..total {
        let v = *d
            .at::<f64>(i as i32)
            .map_err(|e| format!("dist[{i}]: {e}"))?;
        out.push(v);
    }
    return Ok(out);
}

/// 한 프레임 ChArUco 검출 + 오버레이 (인터랙티브 calib용).
#[derive(Debug, Clone)]
pub struct CharucoFrameDetect {
    /// 보정에 쓸 만한 코너 수 (≥ [`MIN_CHARUCO_CORNERS`])
    pub corners: usize,
    pub markers: usize,
    pub ok: bool,
}

/// 프레임당 최소 ChArUco 코너 (저장·보정 후보).
pub const MIN_CHARUCO_CORNERS: usize = 4;

fn make_charuco_detector(
    board_spec: CharucoBoardSpec,
) -> Result<(CharucoBoard, CharucoDetector), String> {
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

/// BGR 프레임에 마커·ChArUco 코너를 그린다. `ok`면 저장 후보.
pub fn detect_and_draw_charuco(
    bgr: &Mat,
    board_spec: CharucoBoardSpec,
) -> Result<(Mat, CharucoFrameDetect), String> {
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
        CharucoFrameDetect {
            corners,
            markers,
            ok,
        },
    ));
}
