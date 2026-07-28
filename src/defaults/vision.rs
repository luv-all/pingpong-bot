//! 공 검출 조립 + 비전 UI — Params [`Default`]가 앱 프리셋.

use anyhow::{Context, Result, bail};

use crate::CameraId;
use crate::defaults::calib::colormask_path;
use crate::detector::{
    ColorContourCascade, ColormaskParams, RoiParams, RoiTrack, Scorer, ScorerParams, fuse,
    load_colormask_set, track,
};

/// fuse scorer motion 가중.
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
            min_circularity: 0.55,
        };
    }
}

impl Default for RoiParams {
    fn default() -> Self {
        return Self {
            k: 3.5,
            pad: 24,
            m: 1.0,
            half_min: 48,
            half_max: 240,
        };
    }
}

/// [`crate::defaults::DEFAULT_COLORMASK_PATH`]에서 캠별 params. 파일·해당 cam 없으면 에러.
pub fn colormask_for(camera_id: CameraId) -> Result<ColormaskParams> {
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

fn assemble(color: ColormaskParams) -> RoiTrack {
    let scorer = ScorerParams::default();
    let cascade = ColorContourCascade::new(color, &scorer);
    let fuse_det = fuse(
        cascade,
        Scorer::from(&scorer).with_motion_weight(MOTION_WEIGHT),
    )
    .with_motion_weight(MOTION_WEIGHT);
    return track(fuse_det, RoiParams::default());
}

/// 본선: 캠별 colormask → contour cascade + ROI track.
pub fn detector_for(camera_id: CameraId) -> Result<RoiTrack> {
    return Ok(assemble(colormask_for(camera_id)?));
}
