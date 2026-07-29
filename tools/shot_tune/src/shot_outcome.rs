//! 한 발 랠리 결과.

#[derive(Debug, Default, Clone, Copy)]
pub struct ShotOutcome {
    /// 들어오는 공이 정상 랠리 샷인가.
    pub incoming_valid: bool,
    /// `plan_best_swing`이 커밋됐는가.
    pub committed: bool,
    pub contact: bool,
    pub returned: bool,
    pub cleared_net: bool,
    /// 리턴이 상대 코트에 떨어졌는가.
    pub returned_in: bool,
    /// commit 창 동안 최선 peak_joint_speed_ratio.
    pub best_peak_ratio: f64,
}
