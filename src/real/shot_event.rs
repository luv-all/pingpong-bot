//! 워커 → 메인 스레드 샷 라이프사이클 이벤트.
//!
//! 용어는 sim(`SimWorld::try_auto_swing`)과 맞춘다: launch → commit → 포기 → end.
//! 실기엔 슈터가 없으므로 launch 대신 [`ShotEvent::Tracking`](EKF가 궤적을 잡은 시점)이 시작이다.
//!
//! 판정 근거 **수치를 이벤트가 직접 들고 온다**. 로그에 문자열 사유만 남으면 벤치에서
//! "얼마나 늦었길래 포기했나"를 되짚을 수 없다.

use pingpong_bot::Point3;
use pingpong_bot::robot;

/// 샷 진행 상황. 메인 스레드가 로그로 찍고 종료 시점을 판단한다.
pub enum ShotEvent {
    /// 준비 완료 — 홈 이동까지 끝난 시작 포즈.
    Armed { pose: robot::Pose },
    /// EKF가 속도까지 시드해 궤적을 잡았다 (샷당 1회).
    Tracking { position: Point3, speed: f64 },
    /// 스윙을 커밋했다. 필드는 sim `"shot: swing commit"`과 동일.
    Committed {
        time_to_impact_secs: f64,
        duration_secs: f64,
        impact: Point3,
        rail_start: f64,
        rail_end: f64,
        peak_joint_speed: f64,
    },
    /// 계획이 관절·토크 한계에 걸렸다 — 모터 보호로 포기 (재시도 없음).
    ///
    /// **유일한 포기 사유다.** "남은 시간이 짧다"로는 포기하지 않는다 (2026-07-31).
    Infeasible { reason: String },
    /// 계획 실패 (재시도 가능) — 진단용.
    PlanFailed { reason: String },
    /// 하드웨어 오류로 중단.
    Failed { reason: String },
    /// 제어 워커가 스윙 완주 + 센터 복귀까지 끝냈다. 항상 마지막.
    Done,
}

impl ShotEvent {
    /// 이 이벤트가 샷을 끝내는가 (메인이 마무리 절차를 시작해야 하는가).
    pub fn ends_shot(&self) -> bool {
        return matches!(
            self,
            Self::Committed { .. } | Self::Infeasible { .. } | Self::Failed { .. }
        );
    }
}
