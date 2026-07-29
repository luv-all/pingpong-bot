use std::sync::Arc;
use std::sync::atomic::Ordering;

use kiss3d::prelude::*;

use super::super::scene::build_table_scene;
use super::super::viewer::{self, SimViewerOptions, lock_world_for_frame};
use crate::ball;
use crate::constants::table;
use crate::constants::viewer::CAMERA_DIST_DEFAULT;
use crate::robot;

use super::ball_only_options::BallOnlyViewerOptions;
use super::host_options::SceneHostOptions;
use super::scene::SimScene;

pub(crate) fn run_scene_host(options: SceneHostOptions) -> Result<(), String> {
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

    let mut ball_visual = options.layers.ball.as_ref().map(|_| {
        if options.ghost_ball {
            ball::Visual::spawn_ghost(&mut scene)
        } else {
            ball::Visual::spawn(&mut scene)
        }
    });
    let mut ball_velocity_visual = options
        .layers
        .ball
        .as_ref()
        .map(|_| ball::VelocityVisual::spawn(&mut scene));

    let mut robot_visual = options
        .layers
        .robot
        .as_ref()
        .map(|_| robot::Visual::spawn(&mut scene, options.urdf.as_deref()));

    let _ = &options.layers.shooter;

    while window.render_3d(&mut scene, &mut camera).await {
        if options.shutdown.load(Ordering::Acquire) {
            break;
        }
        if let (Some(handle), Some(visual)) = (&options.layers.ball, &mut ball_visual) {
            match handle.position() {
                Some(p) => {
                    visual.set_world_position(p);
                    if let Some(vector) = &mut ball_velocity_visual {
                        if let Some(v) = handle.velocity() {
                            vector.set_from_velocity(p, v);
                        } else {
                            vector.hide();
                        }
                    }
                }
                None => {
                    visual.hide();
                    if let Some(vector) = &mut ball_velocity_visual {
                        vector.hide();
                    }
                }
            }
        }
        if let (Some(_), Some(visual), Some(world)) =
            (&options.layers.robot, &mut robot_visual, &options.world)
        {
            if let Some(guard) = lock_world_for_frame(world) {
                visual.sync_from_world(&guard, options.urdf.as_deref());
            }
        }
        if let Some(hook) = &options.ui_hook {
            window.draw_ui(|ctx| {
                if let Ok(mut draw) = hook.lock() {
                    draw.draw_ui(ctx);
                }
            });
        }
    }

    options.shutdown.store(true, Ordering::Release);
    return Ok(());
}

pub(crate) fn run_ball_only_viewer(options: BallOnlyViewerOptions) -> Result<(), String> {
    let scene = SimScene::builder()
        .title("pingpong-bot sim (ball only)")
        .table(options.table)
        .with_ball_handle(ball::Handle::from_shared(options.ball_position))
        .build();
    return scene.run(options.shutdown);
}

/// 풀 sim → [`SimScene`] (패널 on).
pub(crate) fn run_sim_viewer(options: SimViewerOptions) -> Result<(), String> {
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
