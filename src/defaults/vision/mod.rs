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
use crate::vision::detect::{Background, ColorBox, Layer, Picker, Spatial};
use crate::vision::triggers::{Any, PlaneCrossing, SigmaThreshold};
use crate::vision::{Detector, Trigger};

/// 픽셀 정밀 찍기용 loupe 배율.
pub const PIXEL_LOUPE_ZOOM: i32 = 8;
/// loupe 소스 반경 [px].
pub const PIXEL_LOUPE_SRC_HALF: i32 = 7;
/// 필터 불확실성이 먼저 줄지 않아도 궤적 예측을 시작할 탁구대 y 비율.
/// 발사기 쪽 0.75에서 궤적은 생성하지만, 실제 레일·팔 명령은 제어측
/// 위치·속도 불확실성 기준을 통과한 뒤에만 내린다.
///
/// 2026-08-12: 0.85(+`max_impact_sigma` 0.30)로 느슨하게 해서 fly_45~53에
/// `clip-review --all`을 돌려 봤다 — 트리거 시점 관측 수는 늘긴커녕 줄었고
/// (더 일찍 거니 그때까지 쌓인 관측 자체가 적다, 당연한 결과), RMSE는 9개 중
/// 8개에서 더 나빠졌다(예: fly_45 11.9→58.6cm, fly_49 27.2→74.2cm) — 관측이
/// 적은 채로 굳혀서 조건이 나쁜 p0,v0 적합을 그대로 커밋해 버리는 것으로 보임.
/// 되돌림. 병목은 트리거 타이밍이 아니라 다른 데 있다.
pub const ALIGNMENT_TRIGGER_TABLE_Y_FRAC: f64 = 0.75;

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
/// 공간 레이어(비행 부피 밖 끄기)를 맨 앞에 둔다 — 한 번은 뺐었다 (두 카메라가 테이블
/// 끝에서 부피를 정면으로 봐서 화면이 곧 부피였고, keep 이 cam0 86 %·cam1 100 %라 매
/// 프레임 풀프레임 AND 가 아무것도 안 얻었다). 로봇 뒤로 카메라를 옮기면서 프레임에
/// 부피 밖(선 사람 자리)이 크게 들어와, 배경 차분도 못 거르는 피부색이 새기 시작했다 —
/// 그래서 배경 차분보다도 앞, 첫 스텝으로 다시 넣는다.
pub fn cascade(params: &camera::Params) -> Result<Detector> {
    let layers: Vec<Box<dyn Layer>> = vec![
        Box::new(Spatial::from_params(params)?),
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
            y: table::LENGTH_Y * ALIGNMENT_TRIGGER_TABLE_Y_FRAC,
        }),
    ]));
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::Point3;
    use crate::vision::State;

    fn uncertain_state(y: f64) -> State {
        return State {
            t: Duration::from_millis(100),
            position: Point3::new(0.70, y, 1.0),
            velocity: Vector3::new(0.0, -4.0, 0.0),
            sigma_position: Vector3::repeat(10.0),
            sigma_velocity: Vector3::repeat(10.0),
            spin: None,
        };
    }

    #[test]
    fn fallback_trigger_starts_at_seventy_five_percent_table_length() {
        let trigger = trigger();
        assert!(!trigger.ready(&[uncertain_state(table::LENGTH_Y * 0.76)]));
        assert!(trigger.ready(&[uncertain_state(table::LENGTH_Y * 0.74)]));
    }
}
