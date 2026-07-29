use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use super::super::layers::SceneLayers;
use super::super::scene::TableSceneOptions;
use crate::ball;
use crate::robot;
use crate::robot::urdf::UrdfModel;
use crate::shooter;
use crate::sim::physics::world::SimWorld;
use crate::sim::session::controls::SimRuntimeControls;

use super::builder::SimSceneBuilder;
use super::host_options::SceneHostOptions;
use super::run::run_scene_host;
use super::ui_draw::SceneUiHook;

/// 조립된 씬 — 레이어 핸들 IO + [`Self::run`].
///
/// 핸들은 씬이 소유한다. `ball()` / `robot()` / `shooter()`로 같은 인스턴스에
/// 접근하고, clone으로 IO용·실행용을 나누지 않는다.
pub struct SimScene {
    pub(crate) table: TableSceneOptions,
    pub(crate) layers: SceneLayers,
    pub(crate) world: Option<Arc<Mutex<SimWorld>>>,
    pub(crate) controls: Option<Arc<Mutex<SimRuntimeControls>>>,
    pub(crate) urdf: Option<Arc<UrdfModel>>,
    pub(crate) enable_panel: bool,
    pub(crate) ui_hook: Option<SceneUiHook>,
    pub(crate) ghost_ball: bool,
    pub(crate) title: String,
}

impl SimScene {
    pub fn builder() -> SimSceneBuilder {
        return SimSceneBuilder::default();
    }

    pub fn ball(&self) -> Option<&ball::Handle> {
        return self.layers.ball.as_ref();
    }

    pub fn robot(&self) -> Option<&robot::Handle> {
        return self.layers.robot.as_ref();
    }

    pub fn shooter(&self) -> Option<&shooter::Handle> {
        return self.layers.shooter.as_ref();
    }

    /// kiss3d 창을 연다 (블로킹).
    pub fn run(&self, shutdown: Arc<AtomicBool>) -> Result<(), String> {
        return run_scene_host(SceneHostOptions {
            table: self.table.clone(),
            layers: self.layers.clone(),
            shutdown,
            title: self.title.clone(),
            world: self.world.clone(),
            controls: self.controls.clone(),
            urdf: self.urdf.clone(),
            enable_panel: self.enable_panel,
            ui_hook: self.ui_hook.clone(),
            ghost_ball: self.ghost_ball,
        });
    }
}
