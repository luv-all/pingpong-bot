//! 공 검출 조립 + 비전 UI — [`camera::Params`] [`Default`]가 앱 프리셋.

use crate::camera;
use anyhow::{Context, Result, bail};

use crate::camera::Calibration;
use crate::defaults::calib::{calibration_path, colormask_path};
use crate::detector::{
    ColormaskDetector, ColormaskParams, ContourDetector, Detector, FloorEdgeMask, RoiParams,
    Scorer, ScorerParams, load_colormask_set,
};

/// scorer motion 가중.
pub const MOTION_WEIGHT: f64 = 0.5;
/// MotionPrior absdiff 이진화 임계.
pub const MOTION_DIFF_THRESH: f64 = 25.0;
/// 픽셀 정밀 찍기용 loupe 배율.
pub const PIXEL_LOUPE_ZOOM: i32 = 8;
/// loupe 소스 반경 [px].
pub const PIXEL_LOUPE_SRC_HALF: i32 = 7;

impl Default for ScorerParams {
    fn default() -> Self {
        return Self {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            // 0.55 → 0.35 (2026-07-31). **날아가는 공은 원이 아니라 타원이다.**
            //
            // 자동 노출로 34 fps에서 녹화하면(`record_stereo`는 `request_short_exposure`를
            // 부르지 않는다) 5.5 m/s 공이 프레임당 16 cm를 지나 크게 번진다. 색은 여전히
            // 주황이라 colormask는 통과하는데 그다음 원형도에서 떨어지고 있었다.
            //
            // `diag_clip_detection` 스윕 (fly_02, 비행 구간 221~243, 23프레임):
            //
            // | 원형도 | cam0 검출 / 구간밖 | cam1 검출 / 구간밖 |
            // |--------|-------------------|-------------------|
            // | 0.55   | 9 (39%) / 40      | 16 (70%) / 5      |
            // | 0.45   | 15 (65%) / 49     | 20 (87%) / 6      |
            // | 0.35   | 19 (83%) / 74     | 22 (96%) / 8      |
            // | 0.25   | 19 (83%) / 160    | 22 (96%) / 12     |
            //
            // 0.35에서 검출이 크게 오르는데 오검출은 cam1 5→8로 거의 안 는다. 0.25는
            // 더 얻는 것 없이 오검출만 늘어 채택하지 않았다. 색 하한을 푸는 대안은
            // 배경과 맞닿아 있어 위험하다 — cam1 기준 -25에서 구간밖이 5→382로 폭발한다
            // (배경을 공으로 본다). 원형도는 배경이 애초에 원형이 아니라 안전하다.
            //
            // 근본 해결은 녹화 시 짧은 노출 + 실제 120 fps다. 그때 블러가 줄면 이 값을
            // 다시 올릴 수 있다 — 같은 스윕으로 재검증할 것.
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

/// 본선: mask → color → contour → scorer + ROI track.
/// 캘리브·colormask SSOT 필수.
pub fn detector_for(camera_id: camera::Id) -> Result<Detector> {
    let cam = camera_params_for(camera_id)?;
    return assemble(colormask_for(camera_id)?, &cam);
}
