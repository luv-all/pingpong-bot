//! 워커 → 메인 스레드 샷 라이프사이클 이벤트.
//!
//! 용어는 sim(`SimWorld::try_auto_swing`)과 맞춘다: launch → commit → 포기 → end.
//! 실기엔 슈터가 없으므로 launch 대신 [`ShotEvent::Tracking`](EKF가 궤적을 잡은 시점)이 시작이다.
//!
//! 연속 급구에서는 `shot_seq`로 샷을 구분한다. 메인은 Committed/Infeasible로 세션을 끝내지 않는다.

use pingpong_bot::Point3;
use pingpong_bot::robot;

/// 샷 진행 상황. 메인 스레드가 로그로 찍는다 (세션 종료는 ESC/`q` 또는 제어 `Done`).
pub enum ShotEvent {
    /// 준비 완료 — 이번 급구를 받을 수 있는 포즈.
    Armed { shot_seq: u64, pose: robot::Pose },
    /// EKF가 속도까지 시드해 궤적을 잡았다 (샷당 1회).
    Tracking {
        shot_seq: u64,
        position: Point3,
        speed: f64,
    },
    /// 스윙을 커밋했다. 필드는 sim `"shot: swing commit"`과 동일.
    Committed {
        shot_seq: u64,
        time_to_impact_secs: f64,
        duration_secs: f64,
        impact: Point3,
        rail_start: f64,
        rail_end: f64,
        peak_joint_speed: f64,
    },
    /// 계획이 관절·토크 한계에 걸렸다 — 이번 스윙만 포기 (다음 급구는 재시도).
    ///
    /// **유일한 스윙 포기 사유다.** "남은 시간이 짧다"로는 포기하지 않는다 (2026-07-31).
    Infeasible { shot_seq: u64, reason: String },
    /// 계획 실패 (재시도 가능) — 진단용.
    PlanFailed { shot_seq: u64, reason: String },
    /// 하드웨어 오류로 중단 (세션을 끝낼 수 있음).
    Failed { shot_seq: u64, reason: String },
    /// 제어 워커가 루프를 종료했다. 항상 마지막.
    Done,
}
