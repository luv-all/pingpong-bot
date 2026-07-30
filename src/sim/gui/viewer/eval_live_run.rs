//! 합산 Run 시나리오 라이브 재실행.

use crate::sim::eval;

/// 합산 Run으로 저장된 시나리오를 라이브 월드에서 다시 실행·채점.
#[derive(Debug, Clone)]
pub struct EvalLiveRun {
    /// 1..=30 표시용.
    pub shot_number: usize,
    pub zone: eval::Zone,
    pub observer: eval::LiveObserver,
    /// 종료 시 이 실행에서 받은 점수.
    pub live_points: Option<u8>,
    /// 네트 CCD 투과로 채점 무효.
    pub net_passthrough: bool,
}
