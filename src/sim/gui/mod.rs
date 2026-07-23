//! kiss3d / egui 뷰어·디버그 오버레이.

#[cfg(feature = "gui")]
mod mesh_loader;
#[cfg(feature = "gui")]
mod panel;
#[cfg(feature = "gui")]
mod viewer;

pub mod debug_overlays;
pub mod debug_snap;

pub use debug_overlays::DebugOverlays;
pub use debug_snap::{CommitPhase, SimDebugSnapshot};

#[cfg(feature = "gui")]
pub use viewer::{SimViewerOptions, run as run_sim_viewer};
