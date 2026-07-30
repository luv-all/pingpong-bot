//! 제어 워커 → 추정 워커. Recovering 동안 Attempt를 막는다.

/// 연속 급구에서 제어가 커밋을 받을 준비가 됐는지.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlStatus {
    /// 커밋 요청을 받아도 된다. `shot_seq`는 이번 급구 번호(1부터).
    Ready { shot_seq: u64 },
    /// 스윙 완주·센터 복귀 중 — CommitRequest 보내지 말 것.
    Recovering { shot_seq: u64 },
}
