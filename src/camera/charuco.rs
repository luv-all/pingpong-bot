//! OpenCV ChArUco 보드 검출 → Calibration JSON 초안.
//!
//! 완전한 `calibrateCameraCharuco` 인트린식 피팅은 보드 규격·왜곡 모델이
//! 확정된 뒤 채운다. 지금은 보드가 보이는 프레임을 OpenCV로 검증하고
//! 이미지 크기를 반영한 Calibration을 내보낸다.

use std::ffi::OsStr;
use std::fs;
use std::path::Path;

use opencv::core::{Size, Vector};
use opencv::objdetect::{
    CharucoBoard, CharucoDetector, CharucoParameters, DetectorParameters,
    PredefinedDictionaryType, RefineParameters, get_predefined_dictionary,
};
use opencv::prelude::*;
use opencv::{imgcodecs, imgproc};

use super::Calibration;

/// `dir`의 이미지에서 ChArUco를 검출하고 Calibration 초안을 만든다.
pub fn calibrate_charuco_draft(dir: &Path) -> Result<Calibration, String> {
    let dict = get_predefined_dictionary(PredefinedDictionaryType::DICT_4X4_50)
        .map_err(|e| format!("dictionary: {e}"))?;
    let board = CharucoBoard::new_def(Size::new(5, 7), 0.04, 0.02, &dict)
        .map_err(|e| format!("board: {e}"))?;
    let charuco_params = CharucoParameters::default().map_err(|e| format!("charuco_params: {e}"))?;
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

    let mut hits = 0usize;
    let mut image_size = Size::default();

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

        let mut charuco_corners = Vector::<opencv::core::Point2f>::new();
        let mut charuco_ids = Vector::<i32>::new();
        let mut marker_corners = Vector::<Vector<opencv::core::Point2f>>::new();
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
        if charuco_ids.len() >= 4 {
            hits += 1;
        }
    }

    if hits == 0 {
        return Err("ChArUco 코너가 검출된 프레임이 없음".into());
    }

    let mut calib = Calibration::sim(1);
    let cam = &mut calib.cameras[0];
    cam.width = image_size.width.max(1) as u32;
    cam.height = image_size.height.max(1) as u32;
    cam.fx = f64::from(cam.width) * 0.9;
    cam.fy = cam.fx;
    cam.cx = f64::from(cam.width) * 0.5;
    cam.cy = f64::from(cam.height) * 0.5;
    cam.label = Some(format!(
        "charuco-draft hits={hits}/{} (K heuristic; full calibCameraCharuco next)",
        entries.len()
    ));
    return Ok(calib);
}
