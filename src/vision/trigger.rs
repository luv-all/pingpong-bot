//! 예측 궤적을 언제 만들지. 구현은 [`super::triggers`].

use super::contract::State;

/// 예측 궤적을 만들어도 되는 순간인가.
///
/// 엣지가 아니라 레벨 조건이다. 처음 참이 된 순간을 잡는 건 [`super::Ekf`]가 한다.
/// 엣지로 두면 서로 다른 조건이 같은 프레임에 걸리는 일이 없어
/// [`All`](super::triggers::All)이 성립하지 않는다.
pub trait Trigger: Send {
    /// 스윕 결과표 라벨.
    fn name(&self) -> &'static str;

    /// 궤적 전체를 받는 이유는 [`FirstBounce`](super::triggers::FirstBounce)처럼 이력이
    /// 필요한 조건이 있어서다.
    fn ready(&self, measured: &[State]) -> bool;
}
