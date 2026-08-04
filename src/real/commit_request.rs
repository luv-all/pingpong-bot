//! 추정 워커 → 제어 워커 메시지.

use std::time::Instant;

use pingpong_bot::estimator::BallTrajectory;
use pingpong_bot::robot::control::PredictionStage;

/// 최신 공 궤적을 이용해 레일과 손목의 한 단계를 갱신하라는 요청.
pub struct CommitRequest {
    /// EKF가 구분한 공 궤적 번호. 새 번호면 제어 단계 래치를 초기화한다.
    pub track_seq: u64,
    pub trajectory: BallTrajectory,
    /// 초기 목표인지, 0.25 s 관측·10 cm 수렴을 통과한 정밀 목표인지.
    pub stage: PredictionStage,
    /// 예측을 만든 시각.
    pub at: Instant,
}

impl CommitRequest {
    /// 요청이 만들어진 뒤 흐른 시간 [s].
    pub fn age_secs(&self) -> f64 {
        return self.at.elapsed().as_secs_f64();
    }
}
