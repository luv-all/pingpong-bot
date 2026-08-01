//! 추정 워커 → 제어 워커 메시지.

use std::time::Instant;

use pingpong_bot::estimator::BallTrajectory;
use pingpong_bot::robot::control::PredictionStage;

/// "지금 이 공 궤적에서 최적 목표 위치를 계획해 보라"는 요청.
///
/// 제어 워커는 `trajectory.reference_time + target.time_secs`를 절대 만료
/// 시각으로 사용한다. `BallTrajectory`에 타격점은 포함하지 않는다.
pub struct CommitRequest {
    pub trajectory: BallTrajectory,
    /// 초기 목표인지, 0.25 s 관측·10 cm 수렴을 통과한 정밀 목표인지.
    pub stage: PredictionStage,
    /// 요청 시점 EKF 공 x [m].
    pub ball_x: f64,
    /// 요청 시점의 공 y [m] — 로그용.
    pub ball_y: f64,
    /// 요청 시점 EKF x 속도 [m/s].
    pub ball_vx: f64,
    /// 필터 전 최신 삼각측량 x [m]. 최근 150 ms 안의 값만 담는다.
    pub raw_ball_x: Option<f64>,
    /// 예측을 만든 시각.
    pub at: Instant,
}

impl CommitRequest {
    /// 요청이 만들어진 뒤 흐른 시간 [s].
    pub fn age_secs(&self) -> f64 {
        return self.at.elapsed().as_secs_f64();
    }
}
