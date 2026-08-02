//! 비전 산출물(JSON)에서 카메라별 설정을 읽어 검출기를 조립한다.

use anyhow::{Context, Result, bail};

use crate::camera;
use crate::camera::Calibration;
use crate::defaults::calib::{calibration_path, colormask_path};
use crate::vision::Detector;
use crate::vision::detect::colormask::{ColormaskParams, load_colormask_set};

/// 픽셀 정밀 찍기용 loupe 배율.
pub const PIXEL_LOUPE_ZOOM: i32 = 8;
/// loupe 소스 반경 [px].
pub const PIXEL_LOUPE_SRC_HALF: i32 = 7;

/// [`crate::defaults::DEFAULT_COLORMASK_PATH`]에서 캠별 params. 파일·해당 cam 없으면 에러.
pub fn colormask_for(camera_id: camera::Id) -> Result<ColormaskParams> {
    let path = colormask_path();
    let set =
        load_colormask_set(&path).with_context(|| format!("colormask 로드: {}", path.display()))?;
    let Some(params) = set.params(camera_id).cloned() else {
        bail!(
            "{} 에 cam{} 없음 — tune-colormask --cam … 로 저장",
            path.display(),
            camera_id.0
        );
    };
    return Ok(params);
}

/// [`calibration_path`]에서 캠 [`camera::Params`]. 없으면 에러.
pub fn camera_params_for(camera_id: camera::Id) -> Result<camera::Params> {
    let path = calibration_path();
    let text = std::fs::read_to_string(&path)
        .with_context(|| format!("calibration 읽기: {}", path.display()))?;
    let calib: Calibration = serde_json::from_str(&text)
        .with_context(|| format!("calibration JSON: {}", path.display()))?;
    let Some(params) = calib.params(camera_id).cloned() else {
        bail!(
            "{} 에 cam{} 없음 — calib-table-pnp 등으로 저장",
            path.display(),
            camera_id.0
        );
    };
    return Ok(params);
}

/// 본선 캐스케이드. 캘리브·colormask SSOT 필수.
pub fn detector_for(camera_id: camera::Id) -> Result<Detector> {
    return Detector::for_camera(&camera_params_for(camera_id)?);
}
