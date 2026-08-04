//! 추정 워커 → 제어 워커 메시지.

use std::time::Instant;

use pingpong_bot::robot::motion::Prediction;

/// "지금 이 후보들로 스윙을 계획해 보라"는 요청.
///
/// `predictions`의 `time_to_impact_secs`는 [`Self::at`] 시점 기준이다. 제어 워커가 계획을
/// 시작할 때까지 흐른 시간만큼 낡으므로, 오래된 요청은 계획하지 말고 버려야 한다
/// ([`Self::age_secs`]).
pub struct CommitRequest {
    pub predictions: Vec<Prediction>,
    /// 요청 시점의 공 y [m] — 로그용.
    pub ball_y: f64,
    /// 예측을 만든 시각.
    pub at: Instant,
}

impl CommitRequest {
    /// 요청이 만들어진 뒤 흐른 시간 [s].
    pub fn age_secs(&self) -> f64 {
        return self.at.elapsed().as_secs_f64();
    }
}
