use std::sync::Arc;
use std::sync::atomic::Ordering;

use kiss3d::prelude::*;

use super::super::scene::build_table_scene;
use super::super::viewer::{self, SimViewerOptions, lock_world_for_frame};
use crate::constants::table;
use crate::constants::viewer::CAMERA_DIST_DEFAULT;

use super::ball_only_options::BallOnlyViewerOptions;
use super::host_options::SceneHostOptions;
use super::scene::SimScene;
use crate::sim::gui::ball;
use crate::sim::gui::robot;
use crate::sim::gui::shooter;

fn vec3(p: crate::Point3) -> Vec3 {
    return Vec3::new(p.x as f32, p.y as f32, p.z as f32);
}

/// 좌상단 범례. 궤적 색 그대로 칠한 사각형 + 이름.
fn draw_legend(ctx: &kiss3d::egui::Context, entries: &[(&str, Color)]) {
    use kiss3d::egui;

    // 라벨에 한글이 올 수 있다. egui 기본 폰트엔 글리프가 없어서 네모로 나온다.
    super::super::viewer::ensure_korean_fonts(ctx);
    egui::Area::new("trail-legend".into())
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(12.0, 12.0))
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                for (label, color) in entries {
                    ui.horizontal(|ui| {
                        let (rect, _) =
                            ui.allocate_exact_size(egui::vec2(18.0, 4.0), egui::Sense::hover());
                        ui.painter().rect_filled(
                            rect,
                            1.0,
                            egui::Color32::from_rgb(
                                (color.r * 255.0) as u8,
                                (color.g * 255.0) as u8,
                                (color.b * 255.0) as u8,
                            ),
                        );
                        ui.label(*label);
                    });
                }
            });
        });
}

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

    let mut ghost_visual = options
        .layers
        .ghost
        .as_ref()
        .map(|_| ball::Visual::spawn_ghost(&mut scene));

    let mut robot_visual = options
        .layers
        .robot
        .as_ref()
        .map(|_| robot::Visual::spawn(&mut scene, options.urdf.as_deref()));

    let mut shooter_visual = options
        .layers
        .shooter
        .as_ref()
        .map(|_| shooter::Visual::spawn(&mut scene));

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
        if let (Some(handle), Some(visual)) = (&options.layers.ghost, &mut ghost_visual) {
            match handle.position() {
                Some(p) => visual.set_world_position(p),
                None => visual.hide(),
            }
        }
        if let (Some(handle), Some(visual)) = (&options.layers.shooter, &mut shooter_visual) {
            // 메인 뷰어와 같은 SSOT — 물리 월드의 본체 자세. 월드가 없으면 설정에서.
            match options.world.as_ref().and_then(|w| lock_world_for_frame(w)) {
                Some(guard) => {
                    let (pos, rot) = guard.shooter_pose();
                    visual.set_pose(pos, rot);
                }
                None => visual.set_from_settings(&handle.settings()),
            }
        }
        if let (Some(_), Some(visual), Some(world)) =
            (&options.layers.robot, &mut robot_visual, &options.world)
        {
            if let Some(guard) = lock_world_for_frame(world) {
                visual.sync_from_world(&guard, options.urdf.as_deref());
            }
        }
        // 선은 노드가 아니라 프레임 단위 draw call이다 — 매 프레임 다시 그려야 남는다.
        for trail in &options.layers.trails {
            let points = trail.points();
            let (color, width) = (trail.color(), trail.width());
            for pair in points.windows(2) {
                window.draw_line(vec3(pair[0]), vec3(pair[1]), color, width, false);
            }
        }
        // 이름 붙은 궤적이 하나라도 있으면 범례를 띄운다. 색은 궤적 자신이 들고 있어서
        // 범례와 실제 선이 어긋날 수가 없다.
        let legend: Vec<(&str, Color)> = options
            .layers
            .trails
            .iter()
            .filter_map(|trail| Some((trail.label()?, trail.color())))
            .collect();
        if !legend.is_empty() {
            window.draw_ui(|ctx| draw_legend(ctx, &legend));
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
