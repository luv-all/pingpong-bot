//! 실기 워커가 메인 스레드에 보내는 현재 제어 상태.

use std::time::Instant;

use pingpong_bot::Point3;
use pingpong_bot::robot;

use super::TestZone;

/// 프리뷰 상태 패널이 그릴 현재 공 처리 상태 스냅샷.
#[derive(Debug, Clone, Copy)]
pub enum ControlStateSnapshot {
    Idle,
    Aligning {
        track_seq: u64,
        return_due_at: Instant,
        rail_commanded_m: f64,
        aim_commanded_rad: f64,
    },
}

/// 라켓 헤드·레일 단순 제어 런타임 이벤트.
pub enum RuntimeEvent {
    /// 하드웨어 초기화가 끝나 예측 요청을 받을 수 있다.
    Ready { pose: robot::Pose },
    /// 새 `vision::Fit`이 공의 위치와 속도를 추정하기 시작했다.
    Tracking {
        track_seq: u64,
        position: Point3,
        speed: f64,
    },
    /// 한 단계의 레일·라켓 조준 명령을 하드웨어에 전달했다.
    Commanded {
        track_seq: u64,
        target: Point3,
        rail_x: f64,
        aim_rad: f64,
    },
    /// 현재 공 처리 상태가 바뀌었다 — 프리뷰 상태 패널이 소비한다.
    ControlState { state: ControlStateSnapshot },
    /// 준비 자세 레일 x가 존 선택으로 바뀌었거나, 수동 컨트롤로 재적용됐다.
    /// 프리뷰 상태 패널이 소비한다.
    TestZoneChanged { zone: TestZone, home_rail_x: f64 },
    /// 현재 공의 계획 생략 또는 하드웨어 오류. 계획 생략은 다음 공을 계속 처리한다.
    Failed {
        track_seq: Option<u64>,
        reason: String,
    },
    /// 제어 워커가 종료됐다. 항상 마지막 이벤트다.
    Done,
}
