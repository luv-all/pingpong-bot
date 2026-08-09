//! 추정 워커 → 제어 워커 메시지.

use std::time::Instant;

use pingpong_bot::vision::Trajectory;

/// 최신 공 궤적을 이용해 레일과 라켓 조준의 한 단계를 갱신하라는 요청.
pub struct CommitRequest {
    /// 새 비전이 만든 공 하나의 전체 관측·예측 궤적 계약.
    pub trajectory: Trajectory,
    /// 예측을 만든 시각.
    pub at: Instant,
}

impl CommitRequest {
    /// 요청이 만들어진 뒤 흐른 시간 [s].
    pub fn age_secs(&self) -> f64 {
        return self.at.elapsed().as_secs_f64();
    }

    pub fn track_seq(&self) -> u64 {
        return self.trajectory.seq;
    }
}
