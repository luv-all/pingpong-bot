//! 전체 리포트.

use super::{Mode, PASS_SCORE_EXCLUSIVE, Shot, Zone, ZoneScore};

/// 전체 리포트.
#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    pub mode: Mode,
    pub shots: Vec<Shot>,
    pub by_zone: [ZoneScore; 3],
    pub total: u32,
    pub counts: [u32; 4],
}

impl Report {
    pub fn passed(&self) -> bool {
        return self.total > PASS_SCORE_EXCLUSIVE;
    }

    pub fn zone_score(&self, zone: Zone) -> ZoneScore {
        return self.by_zone[zone.zone_index()];
    }
}
