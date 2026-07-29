use std::sync::{Arc, Mutex};

use super::super::layers::SceneLayers;
use super::super::scene::TableSceneOptions;
use crate::ball;
use crate::robot;
use crate::robot::urdf::UrdfModel;
use crate::shooter;
use crate::sim::physics::world::SimWorld;
use crate::sim::session::controls::SimRuntimeControls;

use super::scene::SimScene;
use super::ui_draw::SceneUiHook;

/// [`SimScene`] 조립.
#[derive(Default)]
pub struct SimSceneBuilder {
    table: TableSceneOptions,
    ball: Option<ball::Handle>,
    robot: Option<robot::Handle>,
    shooter: Option<shooter::Handle>,
    world: Option<Arc<Mutex<SimWorld>>>,
    controls: Option<Arc<Mutex<SimRuntimeControls>>>,
    urdf: Option<Arc<UrdfModel>>,
    enable_panel: bool,
    ui_hook: Option<SceneUiHook>,
    ghost_ball: bool,
    title: String,
}

impl SimSceneBuilder {
    pub fn title(mut self, title: impl Into<String>) -> Self {
        self.title = title.into();
        return self;
    }

    pub fn table(mut self, table: TableSceneOptions) -> Self {
        self.table = table;
        return self;
    }

    /// 외부 write용 공 레이어 (`ball::Handle::new`).
    pub fn with_ball(mut self) -> Self {
        self.ball = Some(ball::Handle::new());
        return self;
    }

    pub fn with_ball_handle(mut self, handle: ball::Handle) -> Self {
        self.ball = Some(handle);
        return self;
    }

    /// 월드에 묶인 로봇 레이어. `world`도 씬에 보관한다.
    pub fn with_robot(mut self, world: Arc<Mutex<SimWorld>>) -> Self {
        self.robot = Some(robot::Handle::new(Arc::clone(&world)));
        self.world = Some(world);
        return self;
    }

    pub fn with_robot_handle(mut self, handle: robot::Handle) -> Self {
        if self.world.is_none() {
            self.world = Some(handle.world());
        }
        self.robot = Some(handle);
        return self;
    }

    pub fn with_shooter(
        mut self,
        controls: Arc<Mutex<SimRuntimeControls>>,
        world: Option<Arc<Mutex<SimWorld>>>,
    ) -> Self {
        if let Some(w) = &world {
            if self.world.is_none() {
                self.world = Some(Arc::clone(w));
            }
        }
        self.controls = Some(Arc::clone(&controls));
        self.shooter = Some(shooter::Handle::new(controls, world));
        return self;
    }

    /// 월드 공 동기화 볼 레이어 (메인 sim).
    pub fn with_ball_from_world(mut self, world: Arc<Mutex<SimWorld>>) -> Self {
        if self.world.is_none() {
            self.world = Some(Arc::clone(&world));
        }
        self.ball = Some(ball::Handle::from_world(world));
        return self;
    }

    pub fn urdf(mut self, urdf: Option<Arc<UrdfModel>>) -> Self {
        self.urdf = urdf;
        return self;
    }

    /// 풀 egui 패널 (world·controls 필요). `ui_hook`과 동시에 쓰면 panel이 우선.
    pub fn enable_panel(mut self, enable: bool) -> Self {
        self.enable_panel = enable;
        return self;
    }

    /// 경량 호스트 egui 콜백 (jog 등). `enable_panel(true)`면 무시된다.
    pub fn with_ui_hook(mut self, hook: SceneUiHook) -> Self {
        self.ui_hook = Some(hook);
        return self;
    }

    /// 공을 반투명 홀로그램으로 표시 (도달점 미리보기).
    pub fn ghost_ball(mut self, enable: bool) -> Self {
        self.ghost_ball = enable;
        return self;
    }

    pub fn build(self) -> SimScene {
        let title = if self.title.is_empty() {
            if self.enable_panel {
                "pingpong-bot sim".into()
            } else {
                "pingpong-bot sim (layers)".into()
            }
        } else {
            self.title
        };
        return SimScene {
            table: self.table,
            layers: SceneLayers {
                ball: self.ball,
                robot: self.robot,
                shooter: self.shooter,
            },
            world: self.world,
            controls: self.controls,
            urdf: self.urdf,
            enable_panel: self.enable_panel,
            ui_hook: self.ui_hook,
            ghost_ball: self.ghost_ball,
            title,
        };
    }
}
