//! sim 배치 프리셋.

/// sim 배치 프리셋.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountPreset {
    /// 내장 competition primitive / mesh 없는 URDF
    Competition,
    /// REP-103 Z-up URDF — [`crate::defaults::rail_frame`] 마운트
    Rep103AtTableEnd,
}
