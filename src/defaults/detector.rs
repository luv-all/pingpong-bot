//! 구 `detector` 스택(EKF 기반 제어 경로) 조립 + 프리셋.
//!
//! `vision`(main)과 이름이 겹치는 자리(`detector_for`·`colormask_for`)가 있어 여기서
//! 따로 관리한다 — `real` 제어 경로가 아직 이 스택을 쓴다.

use crate::camera;
use anyhow::{Context, Result, bail};

use crate::defaults::calib::colormask_path;
use crate::defaults::vision::camera_params_for;
use crate::detector::{
    ColormaskDetector, ColormaskParams, ContourDetector, Detector, FloorEdgeMask, RoiParams,
    Scorer, ScorerParams, load_colormask_set,
};

/// scorer motion 가중.
pub const MOTION_WEIGHT: f64 = 0.5;
/// MotionPrior absdiff 이진화 임계.
pub const MOTION_DIFF_THRESH: f64 = 25.0;

impl Default for ScorerParams {
    fn default() -> Self {
        return Self {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.35,
        };
    }
}

impl Default for RoiParams {
    fn default() -> Self {
        return Self {
            radius_scale: 3.5,
            padding: 32,
            motion_scale: 1.0,
            half_min: 48,
            half_max: 320,
        };
    }
}

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

fn assemble(color: ColormaskParams, cam: &camera::Params) -> Result<Detector> {
    let circ = ScorerParams::default().min_circularity;
    let scorer = ScorerParams::from_calib(cam, circ)?;

    return Detector::builder()
        .mask(FloorEdgeMask::from_params(cam)?)
        .then(ColormaskDetector::new(color))
        .then(ContourDetector::from(&scorer))
        .scorer(Scorer::from(&scorer).with_motion_weight(MOTION_WEIGHT))
        .roi(RoiParams::default())
        .build();
}

/// 구 제어 경로 본선: mask → color → contour → scorer + ROI track.
/// 캘리브·colormask SSOT 필수.
pub fn detector_for(camera_id: camera::Id) -> Result<Detector> {
    let cam = camera_params_for(camera_id)?;
    return assemble(colormask_for(camera_id)?, &cam);
}
