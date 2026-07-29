//! 단일 sim 씬 호스트 — [`SimScene`]으로 레이어를 조립하고 핸들로 R/W한다.
//!
//! ```text
//! let scene = SimScene::builder().with_ball().build();
//! scene.ball().unwrap().set_position(Some(p));
//! scene.run(shutdown)?;
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use kiss3d::prelude::*;

use super::layers::{BallHandle, RobotHandle, SceneLayers, ShooterHandle};
use super::scene::{BallVisual, TableSceneOptions, build_table_scene};
use super::viewer::{self, SimViewerOptions};
use crate::Point3;
use crate::constants::table;
use crate::constants::viewer::CAMERA_DIST_DEFAULT;
use crate::robot::urdf::UrdfModel;
use crate::sim::physics::world::SimWorld;
use crate::sim::session::controls::SimRuntimeControls;

/// 조립된 씬 — 레이어 핸들 IO + [`Self::run`].
///
/// 핸들은 씬이 소유한다. `ball()` / `robot()` / `shooter()`로 같은 인스턴스에
/// 접근하고, clone으로 IO용·실행용을 나누지 않는다.
pub struct SimScene {
    table: TableSceneOptions,
    layers: SceneLayers,
    world: Option<Arc<Mutex<SimWorld>>>,
    controls: Option<Arc<Mutex<SimRuntimeControls>>>,
    urdf: Option<Arc<UrdfModel>>,
    enable_panel: bool,
    title: String,
}

impl SimScene {
    pub fn builder() -> SimSceneBuilder {
        return SimSceneBuilder::default();
    }

    pub fn ball(&self) -> Option<&BallHandle> {
        return self.layers.ball.as_ref();
    }

    pub fn robot(&self) -> Option<&RobotHandle> {
        return self.layers.robot.as_ref();
    }

    pub fn shooter(&self) -> Option<&ShooterHandle> {
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
        });
    }
}

/// [`SimScene`] 조립.
#[derive(Default)]
pub struct SimSceneBuilder {
    table: TableSceneOptions,
    ball: Option<BallHandle>,
    robot: Option<RobotHandle>,
    shooter: Option<ShooterHandle>,
    world: Option<Arc<Mutex<SimWorld>>>,
    controls: Option<Arc<Mutex<SimRuntimeControls>>>,
    urdf: Option<Arc<UrdfModel>>,
    enable_panel: bool,
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

    /// 외부 write용 공 레이어 (`BallHandle::new`).
    pub fn with_ball(mut self) -> Self {
        self.ball = Some(BallHandle::new());
        return self;
    }

    pub fn with_ball_handle(mut self, handle: BallHandle) -> Self {
        self.ball = Some(handle);
        return self;
    }

    /// 월드에 묶인 로봇 레이어. `world`도 씬에 보관한다.
    pub fn with_robot(mut self, world: Arc<Mutex<SimWorld>>) -> Self {
        self.robot = Some(RobotHandle::new(Arc::clone(&world)));
        self.world = Some(world);
        return self;
    }

    pub fn with_robot_handle(mut self, handle: RobotHandle) -> Self {
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
        self.shooter = Some(ShooterHandle::new(controls, world));
        return self;
    }

    /// 월드 공 동기화 볼 레이어 (메인 sim).
    pub fn with_ball_from_world(mut self, world: Arc<Mutex<SimWorld>>) -> Self {
        if self.world.is_none() {
            self.world = Some(Arc::clone(&world));
        }
        self.ball = Some(BallHandle::from_world(world));
        return self;
    }

    pub fn urdf(mut self, urdf: Option<Arc<UrdfModel>>) -> Self {
        self.urdf = urdf;
        return self;
    }

    /// 풀 egui 패널 (world·controls 필요).
    pub fn enable_panel(mut self, enable: bool) -> Self {
        self.enable_panel = enable;
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
            title,
        };
    }
}

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
}

/// 레이어 조합에 맞는 kiss3d 창 (블로킹).
pub fn run_scene_host(options: SceneHostOptions) -> Result<(), String> {
    if options.enable_panel {
        let world = options
            .world
            .ok_or_else(|| "enable_panel requires world".to_string())?;
        let controls = options
            .controls
            .ok_or_else(|| "enable_panel requires controls".to_string())?;
        return viewer::run(SimViewerOptions {
            controls,
            world,
            urdf: options.urdf,
            shutdown: options.shutdown,
        });
    }
    return pollster::block_on(run_lightweight(options));
}

async fn run_lightweight(options: SceneHostOptions) -> Result<(), String> {
    let window_attrs = winit::window::WindowAttributes::default().with_title(options.title);
    let mut window = Window::new_with_window_attributes(window_attrs).await;
    let tcx = (table::WIDTH_X * 0.5) as f32;
    let tcy = (table::LENGTH_Y * 0.5) as f32;
    let at = Vec3::new(tcx, tcy, (table::SURFACE_Z * 0.45) as f32);
    let eye = at + Vec3::new(-4.2, -4.0, 3.4);
    let mut camera = OrbitCamera3d::new(eye, at);
    camera.set_up_axis_dir(Vec3::Z);
    camera.set_dist(CAMERA_DIST_DEFAULT);

    let mut scene = SceneNode3d::empty();
    scene
        .add_light(Light::point(80.0))
        .set_position(Vec3::new(2.0, 2.0, 3.0));
    scene
        .add_light(Light::directional(Vec3::new(-0.3, -0.4, -1.0)))
        .set_color(WHITE);

    build_table_scene(&mut scene, &options.table);

    let mut ball_visual = options
        .layers
        .ball
        .as_ref()
        .map(|_| BallVisual::spawn(&mut scene));

    let _ = (
        &options.layers.robot,
        &options.layers.shooter,
        &options.world,
    );

    while window.render_3d(&mut scene, &mut camera).await {
        if options.shutdown.load(Ordering::Acquire) {
            break;
        }
        if let (Some(handle), Some(visual)) = (&options.layers.ball, &mut ball_visual) {
            match handle.position() {
                Some(p) => visual.set_world_position(p),
                None => visual.hide(),
            }
        }
    }

    options.shutdown.store(true, Ordering::Release);
    return Ok(());
}

/// 하위 호환: 공유 슬롯으로 테이블+공 뷰어.
pub struct BallOnlyViewerOptions {
    pub ball_position: Arc<Mutex<Option<Point3>>>,
    pub shutdown: Arc<AtomicBool>,
    pub table: TableSceneOptions,
}

/// [`SimScene`] + 공 슬롯 래퍼.
pub fn run_ball_only_viewer(options: BallOnlyViewerOptions) -> Result<(), String> {
    let scene = SimScene::builder()
        .title("pingpong-bot sim (ball only)")
        .table(options.table)
        .with_ball_handle(BallHandle::from_shared(options.ball_position))
        .build();
    return scene.run(options.shutdown);
}

/// 풀 sim → [`SimScene`] (패널 on).
pub fn run_sim_viewer(options: SimViewerOptions) -> Result<(), String> {
    let world = Arc::clone(&options.world);
    let controls = Arc::clone(&options.controls);
    let scene = SimScene::builder()
        .title("pingpong-bot sim")
        .with_ball_from_world(Arc::clone(&world))
        .with_robot(Arc::clone(&world))
        .with_shooter(controls, Some(world))
        .urdf(options.urdf)
        .enable_panel(true)
        .build();
    return scene.run(options.shutdown);
}
