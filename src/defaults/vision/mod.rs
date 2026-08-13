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

use crate::camera;
use crate::camera::Calibration;
use crate::constants::table;
use crate::defaults::calib::{calibration_path, colormask_path};
use crate::vision::detect::colormask::{ColormaskParams, load_colormask_set};
use crate::vision::detect::{Background, ColorBox, Layer, Picker, Spatial};
use crate::vision::triggers::{All, PlaneCrossing, StereoSamples};
use crate::vision::{Detector, Trigger};

/// 픽셀 정밀 찍기용 loupe 배율.
pub const PIXEL_LOUPE_ZOOM: i32 = 8;
/// loupe 소스 반경 [px].
pub const PIXEL_LOUPE_SRC_HALF: i32 = 7;
/// 예측을 굳힐 수 있는 가장 이른 탁구대 y 비율 — 발사기 쪽 0.75.
///
/// 표본 수([`MIN_STEREO_SAMPLES`])와 **다른 축**을 잰다. 표본 수는 정보량("적합을
/// 세울 만큼 봤나")이고 이건 외삽 거리("여기서 굳히면 접수 평면까지 얼마나 멀리
/// 내다봐야 하나")다. 느린 서브는 공이 아직 저 뒤에 있는데도 짝이 금방 쌓여서,
/// 정보는 충분한데 외삽이 길어 오차가 커진다 — 표본 수만으로는 그걸 못 본다.
pub const ALIGNMENT_TRIGGER_TABLE_Y_FRAC: f64 = 0.75;

/// 예측을 굳히기 전에 **두 캠이 같은 순간에 함께 본** 표본이 최소 몇 개는 있어야 하나.
///
/// 다른 조건(σ·평면)은 전부 적합에서 나온 파생값이라, 적합이 스스로 속으면 같이
/// 속는다. 표본 **개수**는 셈이라 그런 우회가 없어서 하방을 막는 데 쓴다 —
/// [`StereoSamples`] 문서 참고.
///
/// 여기 오기 전에 `PLANE_CROSSING_MAX_VELOCITY_SIGMA`(평면 조건에 속도 σ 상한을
/// `All`로 물린 것)를 먼저 넣었었다. fly_48은 그것도 막았지만, 클립별로 갈라 재 보니
/// 스테레오 하방이 모든 지표에서 이겨서 그쪽을 지웠다 (fly_45~53, 접수 평면 오차
/// 평균 / 최악 / 평균 리드):
///
/// | 하방 | 평균 | 최악 | 리드 |
/// |---|---|---|---|
/// | 없음 | 13.9cm | 51.9cm (fly_52) | 464ms |
/// | σ 게이트 | 7.6cm | 11.5cm | 431ms |
/// | 둘 다 | 7.4cm | 11.5cm | 424ms |
/// | **스테레오 하방** | **6.9cm** | **9.2cm** | **469ms** |
///
/// σ 게이트는 평면 경로를 늦춰 리드타임을 45ms 깎으면서 정확도는 더 나빴다 — 파생값
/// 문턱이라 "적합이 확신하지만 틀린" 국면을 못 걸러서다. 표본 수는 그 국면에서도
/// 정직하게 모자란다.
///
/// **이 값이 곧 리드타임 다이얼이다.** 게이트를 지운 뒤로 트리거를 지배하는 건 이
/// 하나뿐이라(σ를 0.15에서 0.50까지 올려도 결과가 거의 안 변한다 — 평면 경로가 9/9에서
/// 먼저 걸린다), 제어가 시간을 더 원하면 여기를 내리면 된다. `--trigger-sweep` 실측:
///
/// | 표본 | 리드 | 접수 평면 오차 |
/// |---|---|---|
/// | 0 | 520ms | 8.58cm |
/// | 4 | 496ms | 5.67cm |
/// | **6** | **469ms** | **5.40cm** |
/// | 10 | 420ms | 5.45cm |
/// | 15 | 357ms | 4.51cm |
///
/// 0~4 구간이 무릎이다 — 24ms를 내고 2.9cm를 얻는다. 그 뒤는 완만해서 12ms당 0.1cm쯤
/// 이고, 풀링 중앙값의 잡음이 ±0.6cm쯤이라(표의 8 근처가 한 번 되튄다) 6과 4의 차이는
/// 잡음과 비슷한 크기다. 6은 무릎을 막 지난 자리 — 더 벌어야 하면 4가 다음 칸이다.
pub const MIN_STEREO_SAMPLES: usize = 6;

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

/// 본선 트리거 — **두 캠이 같은 순간에 함께 본 표본이 쌓이면** 건다.
///
/// 조건이 이것 하나뿐이다. 원래는 [`Any`]로 σ(빠르면 빠르게)와 [`PlaneCrossing`]
/// (늦어도 반드시)을 묶어 뒀는데, 스테레오 하방을 넣고 재 보니 둘 다 할 일이 없었다:
/// 표본 수가 9/9 클립에서 마지막으로 걸리는 조건이라(즉 늘 이게 제일 늦다) `Any` 안의
/// 둘은 결과를 못 바꾼다. σ는 0.15에서 0.50까지 올려도 표가 거의 안 움직였고, 평면은
/// 표본 수보다 늘 먼저 걸렸다. 안 쓰이는 조건을 조립에 남겨 두면 "이게 뭘 지키고 있나"를
/// 매번 다시 따져야 하므로 지웠다.
///
/// 멀어지는 공은 여기서 안 봐도 된다 — [`Fit`](crate::vision::Fit)이 `velocity.y >= 0`
/// 이면 트랙 자체를 끝낸다. `PlaneCrossing`이 같은 검사를 들고 있었지만 그건 중복이었다.
///
/// 실기와 클립 도구가 **같은 걸** 써야 한다. 도구가 더 늦게 거는 트리거를 쓰면 도구가 재는
/// 리드타임이 실기보다 짧아져, 실기에서 쓸 수 있는 구간을 도구가 못 본다.
pub fn trigger() -> Box<dyn Trigger> {
    return Box::new(All(vec![
        Box::new(StereoSamples {
            min_samples: MIN_STEREO_SAMPLES,
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
    use crate::vision::{Evidence, State};
    use crate::{Point3, Vector3};

    fn state() -> State {
        return State {
            t: Duration::from_millis(100),
            position: Point3::new(0.70, table::LENGTH_Y * 0.3, 1.0),
            velocity: Vector3::new(0.0, -4.0, 0.0),
            sigma_position: Vector3::repeat(0.01),
            sigma_velocity: Vector3::repeat(0.01),
            spin: None,
        };
    }

    /// fly_48·fly_52 회귀 — 적합이 아무리 확신에 차 있어도 두 캠이 같이 본 표본이
    /// 모자라면 안 건다. 이 조건은 파생값이 아니라 셈이라 우회가 없다.
    #[test]
    fn nothing_fires_without_enough_stereo_samples() {
        let trigger = trigger();
        let track = [state()];
        assert!(
            !trigger.ready(&Evidence {
                measured: &track,
                stereo_samples: MIN_STEREO_SAMPLES - 1,
            }),
            "표본이 문턱 하나 모자라면 확신이 서 있어도 안 걸린다"
        );
        assert!(trigger.ready(&Evidence {
            measured: &track,
            stereo_samples: MIN_STEREO_SAMPLES,
        }));
    }
}
