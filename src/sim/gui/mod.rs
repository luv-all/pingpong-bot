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
};
#[cfg(feature = "gui")]
pub use layers::{BallHandle, RobotHandle, SceneLayers, SceneLayersBuilder, ShooterHandle};
#[cfg(feature = "gui")]
pub use robot_visual::{PrimitiveRobotNodes, RobotVisual, UrdfRobotNodes};
#[cfg(feature = "gui")]
pub use scene::{BallVisual, TableSceneOptions};
#[cfg(feature = "gui")]
pub use viewer::{SimViewerOptions, WORLD_LOCK_WAIT};

#[cfg(feature = "gui")]
pub struct SceneHost;

#[cfg(feature = "gui")]
impl SceneHost {
    pub fn run(options: SceneHostOptions) -> Result<(), String> {
        return host::run_scene_host(options);
    }

    pub fn run_ball_only(options: BallOnlyViewerOptions) -> Result<(), String> {
        return host::run_ball_only_viewer(options);
    }

    pub fn build_table_scene(
        scene_root: &mut kiss3d::prelude::SceneNode3d,
        options: &TableSceneOptions,
    ) {
        scene::build_table_scene(scene_root, options);
    }
}

#[cfg(feature = "gui")]
pub struct SimViewer;

#[cfg(feature = "gui")]
impl SimViewer {
    pub fn run(options: SimViewerOptions) -> Result<(), String> {
        return host::run_sim_viewer(options);
    }

    pub fn lock_world_for_frame(
        world: &std::sync::Mutex<crate::sim::physics::world::SimWorld>,
    ) -> Option<std::sync::MutexGuard<'_, crate::sim::physics::world::SimWorld>> {
        return viewer::lock_world_for_frame(world);
    }
}
