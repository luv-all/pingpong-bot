/// 마스터 goal tick을 `2 * zero_tick - master`로 미러하는 슬레이브.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MirrorSlave {
    pub master_id: u8,
    pub slave_id: u8,
}
