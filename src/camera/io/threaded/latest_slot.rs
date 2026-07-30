//! 백그라운드 grab 슬롯의 최신 프레임.

use std::time::Instant;

pub(super) struct LatestSlot {
    pub image: opencv::core::Mat,
    pub timestamp: Instant,
    pub seq: u64,
}
