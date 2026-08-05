//! 비전 튜너블과 조립.
//!
//! `vision` 은 부품만 정의하고 무엇을 어떤 값으로 꽂을지는 정하지 않는다. 캐스케이드
//! 구성도 트리거 조합도 여기가 SSOT 라, 실기와 도구가 갈릴 수 없다.
//!
//! 하위 모듈 경로는 `vision` 트리와 1:1 이다 — [`fit`]은 [`crate::vision::fit`]이,
//! [`detect`]는 [`crate::vision::detect`]가 쓴다. 값은 여기에만 있고 저기서 `use` 한다.

pub mod detect;
pub mod fit;
pub mod seed;

use anyhow::{Context, Result, bail};

use crate::Vector3;
use crate::camera;
use crate::camera::Calibration;
use crate::constants::table;
use crate::defaults::EstimatorParams;
use crate::defaults::calib::{calibration_path, colormask_path};
use crate::vision::detect::colormask::{ColormaskParams, load_colormask_set};
use crate::vision::detect::{Background, ColorBox, Layer, Picker};
use crate::vision::triggers::{Any, PlaneCrossing, SigmaThreshold};
use crate::vision::{Detector, Trigger};

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

/// [`cascade`] 를 캘리브 파일에서 읽은 params 로.
pub fn detector_for(camera_id: camera::Id) -> Result<Detector> {
    return cascade(&camera_params_for(camera_id)?);
}

/// 본선 캐스케이드. 실기 워커도 도구도 여기를 부른다.
///
/// 레이어를 갈아끼울 땐 여기만 고치면 되고, 개수가 변해도 `detect-full` 은 그대로 돈다 —
/// 그 툴은 [`Detector::trace`] 가 주는 이름과 마스크만 그린다.
///
/// 공간 레이어(비행 부피 밖 끄기)는 뺐다. 두 카메라가 테이블 끝에서 부피를 정면으로
/// 보고 있어서 화면이 곧 부피다 — 실측 keep 이 cam0 86 %, cam1 100 % 였다. 매 프레임
/// 풀프레임 AND 를 내고 아무것도 안 얻는다. 카메라를 옆으로 옮기면 다시 볼 값이 있다.
pub fn cascade(params: &camera::Params) -> Result<Detector> {
    let layers: Vec<Box<dyn Layer>> = vec![
        Box::new(Background::new(
            detect::BACKGROUND_HISTORY,
            detect::BACKGROUND_VAR_THRESHOLD,
            detect::BACKGROUND_SCALE,
            detect::BACKGROUND_LEARNING_RATE,
        )?),
        Box::new(ColorBox::load(params.camera_id)?),
    ];
    return Ok(Detector::new(layers, picker(params)?));
}

/// 캐스케이드 종단. 반지름 밴드는 캘리브에서 나오고, 원형도 하한만 여기서 정한다.
pub fn picker(params: &camera::Params) -> Result<Picker> {
    return Picker::from_calib(params, detect::MIN_CIRCULARITY);
}

/// 본선 트리거 — 필터가 좁혀졌거나, 늦어도 네트를 넘으면.
///
/// [`Any`]인 이유는 둘 중 하나만 쓰면 하나를 포기해야 해서다. σ만 보면 검출이 나쁜 샷에서
/// 영영 안 걸리고, 평면만 보면 이미 확신이 선 샷도 네트까지 기다린다.
///
/// 실기와 클립 도구가 **같은 걸** 써야 한다. 도구가 더 늦게 거는 트리거를 쓰면 도구가 재는
/// 리드타임이 실기보다 짧아져, 실기에서 쓸 수 있는 구간을 도구가 못 본다.
pub fn trigger() -> Box<dyn Trigger> {
    let params = EstimatorParams::default();
    let sigma = params.max_impact_sigma;
    return Box::new(Any(vec![
        Box::new(SigmaThreshold {
            position: Vector3::repeat(sigma),
            // 속도 σ는 리드타임을 곱해 도달점 오차가 되므로 같은 예산을 최대 리드로 나눈다.
            velocity: Vector3::repeat(sigma / params.max_lead),
        }),
        Box::new(PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    ]));
}
