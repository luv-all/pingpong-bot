//! 진행 중인 요청 메타데이터.

pub(super) struct Inflight {
    pub(super) id: u64,
    pub(super) requested_at_sim_time: f64,
}
