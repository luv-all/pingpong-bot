//! 예측 궤적을 언제 만들지. 구현은 [`super::triggers`].

use std::ops::Deref;

use super::contract::State;

/// 트리거가 판정에 쓸 수 있는 것 전부.
///
/// 적합된 궤적만 넘기면 **어떤 관측으로 그 궤적이 나왔는지**를 잃는다. 실측 fly_48이
/// 그 구멍이었다: 우캠이 서브 스윙 중 23프레임을 놓쳐 좌캠 단안만 26개 쌓인 채로
/// 트리거가 걸렸는데, 궤적만 보면 관측이 26개나 있는 정상 트랙과 구분이 안 된다.
/// 깊이를 실제로 묶는 건 **두 캠이 같은 순간에 함께 본 것**이라, 그 개수를 따로 넘긴다.
pub struct Evidence<'a> {
    /// 적합된 궤적을 관측 시각마다 표본한 것.
    pub measured: &'a [State],
    /// 두 캠이 **같은 순간에** 함께 본 표본 수.
    ///
    /// 캠별 개수의 최솟값이 아니라 **짝**을 센다 — 한쪽이 여섯 번 봤어도 그게 다
    /// 반대편이 못 본 순간이면 깊이는 하나도 안 묶인다. "같은 순간"의 문턱은 새로
    /// 만들지 않았다: 삼각측량이 이미 쓰는 상호 최근접 짝짓기(스큐 p95 18.9 ms를
    /// 덮는 20 ms)를 그대로 센 것이라, 이 값이 곧 생 궤적 점의 개수다.
    pub stereo_samples: usize,
}

impl<'a> Evidence<'a> {
    /// 스테레오 표본 수를 안 따지는 관측 — 그와 무관한 트리거를 시험할 때 쓴다.
    pub fn of(measured: &'a [State]) -> Self {
        return Self {
            measured,
            stereo_samples: measured.len(),
        };
    }
}

/// 궤적만 보는 트리거가 [`Evidence`]를 슬라이스처럼 쓰게 한다 — 균형을 안 보는 조건은
/// 이 트레이트가 생기기 전과 똑같이 쓰인다.
impl Deref for Evidence<'_> {
    type Target = [State];

    fn deref(&self) -> &[State] {
        return self.measured;
    }
}

/// 예측 궤적을 만들어도 되는 순간인가.
///
/// 엣지가 아니라 레벨 조건이다. 처음 참이 된 순간을 잡는 건 [`super::Fit`]이 한다.
/// 엣지로 두면 서로 다른 조건이 같은 프레임에 걸리는 일이 없어
/// [`All`](super::triggers::All)이 성립하지 않는다.
pub trait Trigger: Send {
    /// 스윕 결과표 라벨.
    fn name(&self) -> &'static str;

    /// 궤적 전체를 받는 이유는 [`FirstBounce`](super::triggers::FirstBounce)처럼 이력이
    /// 필요한 조건이 있어서다.
    fn ready(&self, evidence: &Evidence) -> bool;
}
