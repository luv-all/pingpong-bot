//! 한 발 결과.

use super::{Flags, Zone};
use crate::sim::launch;

/// 한 발 결과.
#[derive(Debug, Clone, PartialEq)]
pub struct Shot {
    pub zone: Zone,
    pub index_in_zone: usize,
    pub flags: Flags,
    pub points: u8,
    /// 발사 당시 설정 — GUI에서 같은 시나리오를 다시 실행할 때 사용.
    pub settings: launch::Settings,
}
