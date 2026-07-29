//! 호스트 실행용 내부 옵션.

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use super::super::layers::SceneLayers;
use super::super::scene::TableSceneOptions;
use super::ui_draw::SceneUiHook;
use crate::robot::urdf::UrdfModel;
use crate::sim::physics::world::SimWorld;
use crate::sim::session::controls::SimRuntimeControls;

/// 호스트 실행용 내부 옵션 ([`SimScene::run`]이 채운다).
pub struct SceneHostOptions {
    pub table: TableSceneOptions,
    pub layers: SceneLayers,
    pub shutdown: Arc<AtomicBool>,
    pub title: String,
    pub world: Option<Arc<Mutex<SimWorld>>>,
    pub controls: Option<Arc<Mutex<SimRuntimeControls>>>,
    pub urdf: Option<Arc<UrdfModel>>,
    pub enable_panel: bool,
    pub ui_hook: Option<SceneUiHook>,
    pub ghost_ball: bool,
}
