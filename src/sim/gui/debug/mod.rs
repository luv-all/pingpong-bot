//! 디버그 오버레이 토글·월드 스냅샷.

pub mod overlays;
pub mod snap;

pub use overlays::DebugOverlays;
pub use snap::{CommitPhase, SimDebugSnapshot};
