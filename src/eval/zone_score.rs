//! 존별 집계.

/// 존별 집계.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ZoneScore {
    pub total: u32,
    pub counts: [u32; 4],
}
