//! 워커 → 메인 스레드 샷 라이프사이클 이벤트.
//!
//! 용어는 sim(`SimWorld::try_auto_swing`)과 맞춘다: launch → commit → 포기 → end.
//! 실기엔 슈터가 없으므로 launch 대신 [`ShotEvent::Tracking`](EKF가 궤적을 잡은 시점)이 시작이다.

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
        rail_end: f64,
        peak_joint_speed: f64,
    },
    /// 이 공은 포기 — 팔 정지. 랠리가 아니므로 곧 종료로 이어진다.
    Abandoned { reason: String },
    /// 계획 실패 (재시도 가능) — 진단용.
    PlanFailed { reason: String },
    /// 하드웨어 오류로 중단.
    Failed { reason: String },
    /// 제어 워커가 스윙 완주 + 센터 복귀까지 끝냈다. 항상 마지막.
    Done,
}

impl ShotEvent {
    /// 이 이벤트가 샷을 끝내는가 (메인이 종료 절차를 시작해야 하는가).
    pub fn ends_shot(&self) -> bool {
        return matches!(
            self,
            Self::Committed { .. } | Self::Abandoned { .. } | Self::Failed { .. }
        );
    }
}
