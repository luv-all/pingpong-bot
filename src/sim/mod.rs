//! Rapier3d 디지털 트윈.
//!
//! - [`physics`]: 탁구대·피더·로봇 라켓·공
//! - [`launch`]: sim 피더 발사 파라미터 SSOT
//! - [`session`]: 물리 스레드 + 공유 월드
//! - [`gui`]: kiss3d 3D + egui (feature `gui`)

pub mod gui;
pub mod launch;
pub mod physics;
pub mod session;

#[cfg(feature = "gui")]
pub use gui::{
    BallOnlyViewerOptions, SceneHost, SceneHostOptions, SceneLayers, SceneUiDraw, SceneUiHook,
    SimScene, SimSceneBuilder, SimViewer, SimViewerOptions, TableSceneOptions,
};
pub use gui::{CommitPhase, DebugOverlays, SimDebugSnapshot};
pub use launch::{Layout, Settings};
pub use physics::{ArmMultibody, BallState, SimWorld};
pub use session::{SimBallEstimator, SimRuntimeControls, SimSession, SimSessionConfig};

pub use gui::debug_overlays;
pub use gui::debug_snap;
pub use physics::arm_bodies;
pub use physics::world;
pub use session::controls;
pub use session::estimator;

/// 하위 호환: `sim::eval_protocol`
pub mod eval_protocol {
    pub use crate::eval::*;
}
