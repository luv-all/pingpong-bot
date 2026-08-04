//! 실기 워커가 메인 스레드에 보내는 현재 제어 상태.

use pingpong_bot::Point3;
use pingpong_bot::robot;
use pingpong_bot::robot::control::PredictionStage;

/// 라켓 헤드·레일 단순 제어 런타임 이벤트.
pub enum RuntimeEvent {
    /// 하드웨어 초기화가 끝나 예측 요청을 받을 수 있다.
    Ready { pose: robot::Pose },
    /// EKF가 새 공의 위치와 속도를 추정하기 시작했다.
    Tracking {
        track_seq: u64,
        position: Point3,
        speed: f64,
    },
    /// 한 단계의 레일·라켓 조준 명령을 하드웨어에 전달했다.
    Commanded {
        track_seq: u64,
        stage: PredictionStage,
        target: Point3,
        rail_x: f64,
        aim_rad: f64,
    },
    /// 하드웨어 오류로 제어 워커가 중단된다.
    Failed {
        track_seq: Option<u64>,
        reason: String,
    },
    /// 제어 워커가 종료됐다. 항상 마지막 이벤트다.
    Done,
}
