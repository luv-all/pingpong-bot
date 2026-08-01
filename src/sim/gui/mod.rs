//! kiss3d / egui — 레이어 조합 씬 호스트.
//!
//! ```text
//! gui/
//!   host/     SimScene builder + run
//!   layers/   Ball / Robot / Shooter R/W
//!   scene/    table + sim::gui::ball::Visual
//!   viewer/   full sim egui (panel, mesh)
//!   debug/    overlays + snap
//! ```
//!
//! ```ignore
//! let scene = SimScene::builder().with_ball().build();
//! scene.ball().unwrap().set_position(Some(p));
//! scene.run(shutdown)?;
//! ```

pub mod ball;
#[cfg(feature = "gui")]
pub mod host;
#[cfg(feature = "gui")]
pub mod layers;
pub mod robot;
#[cfg(feature = "gui")]
pub mod scene;
pub mod shooter;
pub mod trail;
#[cfg(feature = "gui")]
pub mod viewer;

pub mod debug;

pub use debug::{CommitPhase, DebugOverlays, SimDebugSnapshot};

#[cfg(feature = "gui")]
pub use host::{
    BallOnlyViewerOptions, SceneHostOptions, SceneUiDraw, SceneUiHook, SimScene, SimSceneBuilder,
};
#[cfg(feature = "gui")]
pub use layers::{SceneLayers, SceneLayersBuilder};
#[cfg(feature = "gui")]
pub use scene::TableSceneOptions;
#[cfg(feature = "gui")]
pub use viewer::{SimViewerOptions, WORLD_LOCK_WAIT};

#[cfg(feature = "gui")]
mod scene_host;
#[cfg(feature = "gui")]
mod sim_viewer;

#[cfg(feature = "gui")]
pub use scene_host::SceneHost;
#[cfg(feature = "gui")]
pub use sim_viewer::SimViewer;
