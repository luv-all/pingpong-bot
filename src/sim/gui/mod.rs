//! kiss3d / egui — 레이어 조합 씬 호스트.
//!
//! ```text
//! gui/
//!   host/     SimScene builder + run
//!   layers/   Ball / Robot / Shooter R/W
//!   scene/    table + BallVisual
//!   viewer/   full sim egui (panel, mesh)
//!   debug/    overlays + snap
//! ```
//!
//! ```ignore
//! let scene = SimScene::builder().with_ball().build();
//! scene.ball().unwrap().set_position(Some(p));
//! scene.run(shutdown)?;
//! ```

#[cfg(feature = "gui")]
pub mod host;
#[cfg(feature = "gui")]
pub mod layers;
#[cfg(feature = "gui")]
pub mod robot_visual;
#[cfg(feature = "gui")]
pub mod scene;
#[cfg(feature = "gui")]
pub mod viewer;

pub mod debug;

/// 하위 호환 경로 (`sim::gui::debug_overlays`).
pub use debug::overlays as debug_overlays;
/// 하위 호환 경로 (`sim::gui::debug_snap`).
pub use debug::snap as debug_snap;

pub use debug::{CommitPhase, DebugOverlays, SimDebugSnapshot};

#[cfg(feature = "gui")]
pub use host::{
    BallOnlyViewerOptions, SceneHostOptions, SceneUiDraw, SceneUiHook, SimScene, SimSceneBuilder,
    run_ball_only_viewer, run_scene_host, run_sim_viewer,
};
#[cfg(feature = "gui")]
pub use robot_visual::{PrimitiveRobotNodes, RobotVisual, UrdfRobotNodes};
#[cfg(feature = "gui")]
pub use layers::{BallHandle, RobotHandle, SceneLayers, SceneLayersBuilder, ShooterHandle};
#[cfg(feature = "gui")]
pub use scene::{BallVisual, TableSceneOptions, build_table_scene};
#[cfg(feature = "gui")]
pub use viewer::{SimViewerOptions, WORLD_LOCK_WAIT, lock_world_for_frame};
