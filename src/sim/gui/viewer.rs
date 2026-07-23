//! kiss3d 3D + egui — Rapier sim 월드와 슈터 패널 (macOS: 메인 스레드 단일 창).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::SwingPlanError;
use crate::constants::viewer::{CAMERA_DIST_MAX, CAMERA_DIST_MIN, HIT_PLANE_WALL_HEIGHT};
use crate::constants::{ball, table};
use kiss3d::prelude::*;
use rapier3d::prelude::{Rotation, Vector};
use tracing::info;

use super::debug_overlays::{DebugOverlays, colors};
use super::debug_snap::CommitPhase;
use super::mesh_loader;
use super::panel;
use crate::robot::urdf::{UrdfLinkVisual, UrdfModel};
use crate::sim::session::controls::SimRuntimeControls;
use crate::sim::physics::shooter::{RANDOM_SHOT_TARGET_PADDING_M, ShooterLayout};
use crate::sim::physics::world::SimWorld;

const HIDDEN: Vec3 = Vec3::new(0.0, 0.0, -10.0);
const ARC_NODE_COUNT: usize = 48;
const GHOST_NODE_COUNT: usize = 32;
const OBB_NODE_COUNT: usize = 8;

/// sim 3D + 제어 패널 옵션.
pub struct SimViewerOptions {
    /// 발사·sim 설정
    pub controls: Arc<Mutex<SimRuntimeControls>>,
    /// 공유 sim 월드
    pub world: Arc<Mutex<SimWorld>>,
    /// URDF 모델 (kiss3d 로봇 mesh 대신 사용)
    pub urdf: Option<Arc<crate::robot::urdf::UrdfModel>>,
    /// 창 닫을 때 파이프라인 종료
    pub shutdown: Arc<AtomicBool>,
}

/// kiss3d 창을 메인 스레드에서 연다 (블로킹).
pub fn run(options: SimViewerOptions) -> Result<(), String> {
    return pollster::block_on(viewer_main(options));
}

struct DynamicNodes {
    /// 블레이드 원판 (면 중심 = EE)
    racket: SceneNode3d,
    /// 손목→면 손잡이
    racket_handle: SceneNode3d,
    arm_base: SceneNode3d,
    links: Vec<SceneNode3d>,
    /// `links[i]` 반지름 [m]. `place_link`가 local_scale을 통째로 덮어쓰므로 보관.
    link_radii: Vec<f32>,
    joints: Vec<SceneNode3d>,
    link_color: Color,
    joint_color: Color,
}

struct UrdfVisualNode {
    link_name: String,
    local_pos: Vec3,
    local_rot: Quat,
    node: SceneNode3d,
    base_color: Color,
}

enum RobotRender {
    Primitive(DynamicNodes),
    Urdf(Vec<UrdfVisualNode>),
}

struct SceneDynamics {
    ball: SceneNode3d,
    ball_color: Color,
    shooter: SceneNode3d,
    robot: RobotRender,
    /// hit plane 예측 3D 위치 (공 비행 중 디버그)
    impact_marker: SceneNode3d,
    /// 접수 평면 위 투영 (x, y)
    impact_ring: SceneNode3d,
    /// 접수 평면 반투명 벽 (y = hit plane)
    hit_plane_wall: SceneNode3d,
    fail_x: SceneNode3d,
    aim_band: SceneNode3d,
    rail_min: SceneNode3d,
    rail_max: SceneNode3d,
    rail_cur: SceneNode3d,
    arc_pred: Vec<SceneNode3d>,
    arc_truth: Vec<SceneNode3d>,
    ghost: Vec<SceneNode3d>,
    obbs: Vec<SceneNode3d>,
    omega_shaft: SceneNode3d,
    omega_tip: SceneNode3d,
}

async fn viewer_main(options: SimViewerOptions) -> Result<(), String> {
    let mut window = Window::new("pingpong-bot sim").await;
    let tcx = (table::WIDTH_X * 0.5) as f32;
    let tcy = (table::LENGTH_Y * 0.5) as f32;
    let mut camera = OrbitCamera3d::new(
        Vec3::new(tcx, tcy * 0.35, 2.8),
        Vec3::new(tcx, tcy, table::SURFACE_Z as f32),
    );
    camera.set_up_axis_dir(Vec3::Z);
    let mut scene = SceneNode3d::empty();
    scene
        .add_light(Light::point(80.0))
        .set_position(Vec3::new(2.0, 2.0, 3.0));
    scene
        .add_light(Light::directional(Vec3::new(-0.3, -0.4, -1.0)))
        .set_color(WHITE);

    build_static_scene(&mut scene);
    let mut dynamic = build_scene_dynamics(&mut scene, options.urdf.as_deref());

    let controls = Arc::clone(&options.controls);
    let mut ui_state =
        panel::PanelUiState::from_controls(&options.controls.lock().expect("controls"));
    ui_state.camera_dist = camera.dist().clamp(CAMERA_DIST_MIN, CAMERA_DIST_MAX);

    info!("kiss3d sim — 창: Shooter/View/Status · 축 RGB · debug overlays");

    let mut status_cache = None;
    while window.render_3d(&mut scene, &mut camera).await {
        if options.shutdown.load(Ordering::Acquire) {
            break;
        }
        if let Ok(snapshot) = options.world.try_lock() {
            sync_scene_dynamics(
                &mut dynamic,
                &snapshot,
                options.urdf.as_deref(),
                &ui_state.debug,
            );
            status_cache = Some(panel::StatusSnapshot::from_world(&snapshot));
        }

        let dist_before_ui = ui_state.camera_dist;
        window.draw_ui(|ctx| {
            panel::draw(ctx, &mut ui_state, &controls, status_cache.as_ref());
        });
        if (ui_state.camera_dist - dist_before_ui).abs() > f32::EPSILON {
            camera.set_dist(ui_state.camera_dist.clamp(CAMERA_DIST_MIN, CAMERA_DIST_MAX));
        } else {
            ui_state.camera_dist = camera.dist().clamp(CAMERA_DIST_MIN, CAMERA_DIST_MAX);
        }
    }

    options.shutdown.store(true, Ordering::Release);
    return Ok(());
}

fn build_static_scene(scene: &mut SceneNode3d) {
    let tcx = (table::WIDTH_X * 0.5) as f32;
    let tcy = (table::LENGTH_Y * 0.5) as f32;
    let table_z = (table::SURFACE_Z - table::HALF_THICKNESS) as f32;
    scene
        .add_cube(
            table::WIDTH_X as f32,
            table::LENGTH_Y as f32,
            table::HALF_THICKNESS as f32 * 2.0,
        )
        .set_color(Color::new(0.05, 0.45, 0.18, 1.0))
        .set_position(Vec3::new(tcx, tcy, table_z));

    scene
        .add_cube(table::WIDTH_X as f32, 0.01, table::NET_HEIGHT as f32)
        .set_color(Color::new(0.9, 0.9, 0.92, 0.85))
        .set_position(Vec3::new(
            tcx,
            tcy,
            (table::SURFACE_Z + table::NET_HEIGHT * 0.5) as f32,
        ));

    scene
        .add_cube(
            table::WIDTH_X as f32 * 1.2,
            table::LENGTH_Y as f32 * 1.2,
            0.02,
        )
        .set_color(Color::new(0.25, 0.25, 0.28, 1.0))
        .set_position(Vec3::new(tcx, tcy, 0.01));

    let frame = crate::defaults::rail_frame();
    let rail_y = frame.mount_y() as f32;
    let rail_h = crate::constants::geometry::RAIL_VISUAL_HEIGHT as f32;
    let rail_w = crate::constants::geometry::RAIL_VISUAL_WIDTH as f32;
    let rail_z = frame.mount_z() as f32 - rail_h * 0.5;
    scene
        .add_cube(table::WIDTH_X as f32, rail_w, rail_h)
        .set_color(Color::new(0.35, 0.38, 0.42, 1.0))
        .set_position(Vec3::new((table::WIDTH_X * 0.5) as f32, rail_y, rail_z));

    // 테이블 로봇쪽 코너 윗면 = 월드 (0, 0, SURFACE_Z)
    let axis_origin = Vec3::new(0.0, 0.0, table::SURFACE_Z as f32);
    add_axis_arrow(
        scene,
        axis_origin,
        Vec3::X,
        Color::new(0.95, 0.2, 0.15, 1.0),
    );
    add_axis_arrow(
        scene,
        axis_origin,
        Vec3::Y,
        Color::new(0.2, 0.85, 0.25, 1.0),
    );
    add_axis_arrow(
        scene,
        axis_origin,
        Vec3::Z,
        Color::new(0.25, 0.45, 1.0, 1.0),
    );
}

fn add_axis_arrow(scene: &mut SceneNode3d, origin: Vec3, direction: Vec3, color: Color) {
    let dir = direction.normalize();
    let length = 0.32_f32;
    let tip_h = 0.07_f32;
    let shaft_h = length - tip_h;
    let rot = Quat::from_rotation_arc(Vec3::Y, dir);
    scene
        .add_cylinder(0.010, shaft_h)
        .set_color(color)
        .set_position(origin + dir * (shaft_h * 0.5))
        .set_rotation(rot);
    scene
        .add_cone(0.024, tip_h)
        .set_color(color)
        .set_position(origin + dir * (shaft_h + tip_h * 0.5))
        .set_rotation(rot);
}

fn rgba(c: [f32; 4]) -> Color {
    return Color::new(c[0], c[1], c[2], c[3]);
}

fn build_scene_dynamics(scene: &mut SceneNode3d, urdf: Option<&UrdfModel>) -> SceneDynamics {
    let ball_color = Color::new(1.0, 0.55, 0.05, 1.0);
    let ball = scene.add_sphere(ball::RADIUS as f32).set_color(ball_color);
    let shooter = scene
        .add_cube(
            ShooterLayout::VISUAL_SIZE_X as f32,
            ShooterLayout::VISUAL_SIZE_Y as f32,
            ShooterLayout::VISUAL_SIZE_Z as f32,
        )
        .set_color(Color::new(0.45, 0.45, 0.5, 1.0));
    let impact_marker = scene.add_sphere(0.018).set_color(rgba(colors::IDLE_PRED));
    let impact_ring = scene
        .add_cube(0.05, 0.05, 0.004)
        .set_color(Color::new(1.0, 0.95, 0.1, 0.9));
    let hit_plane_wall = scene
        .add_cube(table::WIDTH_X as f32, 0.004, HIT_PLANE_WALL_HEIGHT)
        .set_color(Color::new(0.55, 0.75, 1.0, 0.14))
        .set_position(Vec3::new(
            (table::WIDTH_X * 0.5) as f32,
            table::DEFAULT_HIT_PLANE_Y as f32,
            table::SURFACE_Z as f32 + HIT_PLANE_WALL_HEIGHT * 0.5,
        ));

    // 도달 밖 X: 교차하는 얇은 막대 2개
    let fail_x = scene
        .add_cube(0.06, 0.008, 0.008)
        .set_color(rgba(colors::UNREACHABLE_X))
        .set_position(HIDDEN);

    let pad = RANDOM_SHOT_TARGET_PADDING_M as f32;
    let band_w = (table::WIDTH_X as f32 - 2.0 * pad).max(0.05);
    let aim_band = scene
        .add_cube(band_w, 0.04, 0.01)
        .set_color(rgba(colors::AIM_BAND))
        .set_position(Vec3::new(
            (table::WIDTH_X * 0.5) as f32,
            0.02,
            (table::SURFACE_Z + 0.006) as f32,
        ));

    let rail_min = scene
        .add_cube(0.02, 0.06, 0.08)
        .set_color(Color::new(0.9, 0.4, 0.2, 0.85))
        .set_position(HIDDEN);
    let rail_max = scene
        .add_cube(0.02, 0.06, 0.08)
        .set_color(Color::new(0.9, 0.4, 0.2, 0.85))
        .set_position(HIDDEN);
    let rail_cur = scene
        .add_cube(0.025, 0.05, 0.05)
        .set_color(Color::new(0.2, 0.9, 0.95, 0.9))
        .set_position(HIDDEN);

    let arc_pred: Vec<_> = (0..ARC_NODE_COUNT)
        .map(|_| {
            scene
                .add_sphere(0.008)
                .set_color(rgba(colors::PRED_ARC))
                .set_position(HIDDEN)
        })
        .collect();
    let arc_truth: Vec<_> = (0..ARC_NODE_COUNT)
        .map(|_| {
            scene
                .add_sphere(0.007)
                .set_color(rgba(colors::TRUTH_ARC))
                .set_position(HIDDEN)
        })
        .collect();
    let ghost: Vec<_> = (0..GHOST_NODE_COUNT)
        .map(|_| {
            scene
                .add_sphere(0.012)
                .set_color(rgba(colors::GHOST))
                .set_position(HIDDEN)
        })
        .collect();
    let obbs: Vec<_> = (0..OBB_NODE_COUNT)
        .map(|_| {
            scene
                .add_cube(1.0, 1.0, 1.0)
                .set_color(rgba(colors::OBB_HIT))
                .set_position(HIDDEN)
        })
        .collect();

    let omega_shaft = scene
        .add_cylinder(0.006, 0.12)
        .set_color(Color::new(0.4, 0.85, 1.0, 0.9))
        .set_position(HIDDEN);
    let omega_tip = scene
        .add_cone(0.014, 0.04)
        .set_color(Color::new(0.4, 0.85, 1.0, 0.9))
        .set_position(HIDDEN);

    let robot = if let Some(model) = urdf {
        RobotRender::Urdf(build_urdf_nodes(scene, model))
    } else {
        RobotRender::Primitive(build_primitive_robot_nodes(scene))
    };

    return SceneDynamics {
        ball,
        ball_color,
        shooter,
        robot,
        impact_marker,
        impact_ring,
        hit_plane_wall,
        fail_x,
        aim_band,
        rail_min,
        rail_max,
        rail_cur,
        arc_pred,
        arc_truth,
        ghost,
        obbs,
        omega_shaft,
        omega_tip,
    };
}

fn build_primitive_robot_nodes(scene: &mut SceneNode3d) -> DynamicNodes {
    use crate::constants::geometry::{
        ARM_BASE_HEIGHT, ARM_BASE_RADIUS, JOINT_MARKER_RADIUS, LINK_FOREARM_RADIUS,
        LINK_UPPER_RADIUS, RACKET_BLADE_RADIUS, RACKET_HALF_Z, RACKET_HANDLE_RADIUS,
    };

    let link_color = Color::new(0.25, 0.45, 0.85, 1.0);
    let joint_color = Color::new(0.95, 0.85, 0.1, 1.0);
    let link_radii = [
        LINK_UPPER_RADIUS,
        LINK_UPPER_RADIUS,
        LINK_FOREARM_RADIUS,
        LINK_FOREARM_RADIUS,
        LINK_FOREARM_RADIUS,
    ];
    let links = link_radii
        .iter()
        .map(|&radius| scene.add_cylinder(radius as f32, 1.0).set_color(link_color))
        .collect();
    let joints = (0..link_radii.len())
        .map(|_| {
            scene
                .add_sphere(JOINT_MARKER_RADIUS as f32)
                .set_color(joint_color)
        })
        .collect();

    return DynamicNodes {
        racket: scene
            .add_cylinder(RACKET_BLADE_RADIUS as f32, (RACKET_HALF_Z * 2.0) as f32)
            .set_color(Color::new(0.85, 0.15, 0.12, 1.0)),
        racket_handle: scene
            .add_cylinder(RACKET_HANDLE_RADIUS as f32, 1.0)
            .set_color(Color::new(0.55, 0.55, 0.58, 1.0)),
        arm_base: scene
            .add_cylinder(ARM_BASE_RADIUS as f32, ARM_BASE_HEIGHT as f32)
            .set_color(Color::new(0.2, 0.25, 0.55, 1.0)),
        links,
        link_radii: link_radii.iter().map(|&r| r as f32).collect(),
        joints,
        link_color,
        joint_color,
    };
}

fn build_urdf_nodes(scene: &mut SceneNode3d, urdf: &UrdfModel) -> Vec<UrdfVisualNode> {
    return urdf
        .link_visuals()
        .into_iter()
        .map(|mut vis| {
            // URDF는 silver 일변도라 시뮬에서 구분 안 됨 → primitive와 비슷한 틴트.
            let rgba = urdf_link_tint(&vis.link_name);
            vis.color = rgba;
            let link_name = vis.link_name.clone();
            let base_color = Color::new(rgba[0], rgba[1], rgba[2], rgba[3]);
            UrdfVisualNode {
                link_name,
                local_pos: rpy_xyz_to_pos(&vis.origin_xyz),
                local_rot: rpy_to_quat(vis.origin_rpy),
                node: add_urdf_visual(scene, &vis),
                base_color,
            }
        })
        .collect();
}

/// primitive 로봇과 비슷한 역할별 색 (서보=노랑, 링크=파랑, 라켓=빨강).
fn urdf_link_tint(link_name: &str) -> [f32; 4] {
    let name = link_name.to_ascii_lowercase();
    if name.contains("paddle") || (name.contains("racket") && !name.contains("joint")) {
        return [0.85, 0.15, 0.12, 1.0]; // 라켓면
    }
    if name.contains("racket_joint") {
        return [0.55, 0.55, 0.58, 1.0]; // 손잡이
    }
    if name.contains("mx-") || name.contains("mx_") || name.contains("dynamixel") {
        return [0.95, 0.85, 0.1, 1.0]; // 서보(조인트)
    }
    if name == "base_link" || name.starts_with("base") {
        return [0.2, 0.25, 0.55, 1.0]; // 베이스
    }
    if name.contains("arm") {
        return [0.25, 0.45, 0.85, 1.0]; // 상완·전완
    }
    if name.contains("fr0") || name.contains("horn") || name.contains("bracket") {
        return [0.35, 0.55, 0.9, 1.0]; // 브라켓·혼
    }
    return [0.45, 0.55, 0.75, 1.0]; // 기타 링크
}

fn add_urdf_visual(scene: &mut SceneNode3d, vis: &UrdfLinkVisual) -> SceneNode3d {
    let color = Color::new(vis.color[0], vis.color[1], vis.color[2], vis.color[3]);
    return mesh_loader::add_geometry(scene, &vis.geometry, color);
}

fn sync_scene_dynamics(
    nodes: &mut SceneDynamics,
    world: &SimWorld,
    urdf: Option<&UrdfModel>,
    debug: &DebugOverlays,
) {
    let ball = world.ball_position();
    nodes.ball.set_position(to_vec3(ball));

    let net_fail = debug.net_gate && world.debug_snap().net_gate_ok == Some(false);
    if net_fail {
        nodes.ball.set_color(rgba(colors::NET_FAIL));
    } else {
        nodes.ball.set_color(nodes.ball_color);
    }

    let (sh_pos, sh_rot) = world.shooter_pose();
    nodes
        .shooter
        .set_position(to_vec3(sh_pos))
        .set_rotation(to_quat(sh_rot));

    sync_impact_debug_markers(nodes, world, debug);
    sync_unreachable_x(nodes, world, debug);
    sync_aim_band(nodes, debug);
    sync_rail_stroke(nodes, world, debug);
    sync_arc_nodes(
        &mut nodes.arc_pred,
        &world.debug_snap().predicted_arc,
        debug.predicted_arc,
    );
    sync_arc_nodes(
        &mut nodes.arc_truth,
        &world.debug_snap().truth_arc,
        debug.truth_arc,
    );
    sync_arc_nodes(
        &mut nodes.ghost,
        &world.debug_snap().committed_racket_path,
        debug.swing_ghost,
    );
    sync_obbs(nodes, world, debug);
    sync_omega_arrow(nodes, world, debug);

    match (&mut nodes.robot, urdf) {
        (RobotRender::Primitive(arm_nodes), None) => {
            sync_primitive_robot(arm_nodes, world, debug);
        }
        (RobotRender::Urdf(urdf_nodes), Some(model)) => {
            let joints = world
                .urdf_joint_values()
                .unwrap_or_else(|| world.robot().joints().values.clone());
            sync_urdf_robot(
                urdf_nodes,
                model,
                joints.as_slice(),
                world.effective_sim_mount(),
                world,
                debug,
            );
        }
        _ => {}
    }
}

fn impact_marker_color(world: &SimWorld) -> Color {
    let snap = world.debug_snap();
    if world.swing_committed() || snap.commit_phase == CommitPhase::Committed {
        return rgba(colors::SUCCESS);
    }
    if let Some(err) = &snap.last_fail {
        return match err {
            SwingPlanError::InverseKinematicsNoSolution { .. } => rgba(colors::IK),
            SwingPlanError::InsufficientTime { .. } => rgba(colors::TIME),
            SwingPlanError::ReturnVelocityUnreachable { .. } => rgba(colors::RETURN),
            SwingPlanError::TablePenetration { .. } => rgba(colors::PENETRATION),
            SwingPlanError::JointOrTorqueLimit { .. } => rgba(colors::LIMIT),
        };
    }
    if world.swing_abandoned() {
        return rgba(colors::IK);
    }
    return rgba(colors::IDLE_PRED);
}

fn sync_impact_debug_markers(nodes: &mut SceneDynamics, world: &SimWorld, debug: &DebugOverlays) {
    if !debug.impact_markers {
        nodes.impact_marker.set_visible(false).set_position(HIDDEN);
        nodes.impact_ring.set_visible(false).set_position(HIDDEN);
        nodes.hit_plane_wall.set_visible(false).set_position(HIDDEN);
        return;
    }
    nodes.impact_marker.set_visible(true);
    nodes.impact_ring.set_visible(true);
    nodes.hit_plane_wall.set_visible(true);

    let plane_y = world
        .debug_prediction()
        .map(|p| p.impact_position.coords.y as f32)
        .unwrap_or(table::DEFAULT_HIT_PLANE_Y as f32);
    let wall_z = table::SURFACE_Z as f32 + HIT_PLANE_WALL_HEIGHT * 0.5;
    nodes
        .hit_plane_wall
        .set_position(Vec3::new((table::WIDTH_X * 0.5) as f32, plane_y, wall_z));

    let marker_color = impact_marker_color(world);
    let Some(pred) = world.debug_prediction() else {
        nodes.impact_marker.set_position(HIDDEN);
        nodes.impact_ring.set_position(HIDDEN);
        nodes
            .hit_plane_wall
            .set_color(Color::new(0.55, 0.75, 1.0, 0.14));
        return;
    };
    let mut wall = marker_color;
    wall.a = 0.32;
    nodes.hit_plane_wall.set_color(wall);
    nodes.impact_marker.set_color(marker_color);
    let p = pred.impact_position.coords;
    nodes.impact_ring.set_position(Vec3::new(
        p.x as f32,
        p.y as f32,
        (table::SURFACE_Z + 0.008) as f32,
    ));
    let marker_z = (p.z as f32).max((table::SURFACE_Z + 0.02) as f32);
    nodes
        .impact_marker
        .set_position(Vec3::new(p.x as f32, p.y as f32, marker_z));
}

fn sync_unreachable_x(nodes: &mut SceneDynamics, world: &SimWorld, debug: &DebugOverlays) {
    let Some([x, y, z]) = world
        .debug_snap()
        .unreachable_xyz
        .filter(|_| debug.unreachable_x)
    else {
        nodes.fail_x.set_visible(false).set_position(HIDDEN);
        return;
    };
    nodes
        .fail_x
        .set_visible(true)
        .set_color(rgba(colors::UNREACHABLE_X))
        .set_position(Vec3::new(x as f32, y as f32, z as f32))
        .set_rotation(Quat::from_rotation_arc(
            Vec3::X,
            Vec3::new(1.0, 1.0, 0.0).normalize(),
        ));
}

fn sync_aim_band(nodes: &mut SceneDynamics, debug: &DebugOverlays) {
    nodes.aim_band.set_visible(debug.aim_band);
    if !debug.aim_band {
        nodes.aim_band.set_position(HIDDEN);
        return;
    }
    let pad = RANDOM_SHOT_TARGET_PADDING_M as f32;
    nodes.aim_band.set_position(Vec3::new(
        (table::WIDTH_X * 0.5) as f32,
        0.02,
        (table::SURFACE_Z + 0.006) as f32,
    ));
    let _ = pad;
}

fn sync_rail_stroke(nodes: &mut SceneDynamics, world: &SimWorld, debug: &DebugOverlays) {
    if !debug.rail_stroke {
        nodes.rail_min.set_visible(false).set_position(HIDDEN);
        nodes.rail_max.set_visible(false).set_position(HIDDEN);
        nodes.rail_cur.set_visible(false).set_position(HIDDEN);
        return;
    }
    let Some(rail) = world.arm().rail.as_ref() else {
        nodes.rail_min.set_visible(false).set_position(HIDDEN);
        nodes.rail_max.set_visible(false).set_position(HIDDEN);
        nodes.rail_cur.set_visible(false).set_position(HIDDEN);
        return;
    };
    let frame = crate::defaults::rail_frame();
    let y = frame.mount_y() as f32;
    let z = frame.mount_z() as f32;
    nodes
        .rail_min
        .set_visible(true)
        .set_position(Vec3::new(rail.x_min as f32, y, z));
    nodes
        .rail_max
        .set_visible(true)
        .set_position(Vec3::new(rail.x_max as f32, y, z));
    nodes.rail_cur.set_visible(true).set_position(Vec3::new(
        world.robot().rail_x() as f32,
        y,
        z + 0.04,
    ));
}

fn sync_arc_nodes(nodes: &mut [SceneNode3d], pts: &[[f64; 3]], enabled: bool) {
    if !enabled {
        for n in nodes.iter_mut() {
            n.set_visible(false).set_position(HIDDEN);
        }
        return;
    }
    let step = if pts.len() <= nodes.len() {
        1
    } else {
        pts.len() / nodes.len()
    }
    .max(1);
    let mut ni = 0;
    for (i, p) in pts.iter().enumerate() {
        if i % step != 0 {
            continue;
        }
        if ni >= nodes.len() {
            break;
        }
        nodes[ni]
            .set_visible(true)
            .set_position(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
        ni += 1;
    }
    for n in nodes.iter_mut().skip(ni) {
        n.set_visible(false).set_position(HIDDEN);
    }
}

fn sync_obbs(nodes: &mut SceneDynamics, world: &SimWorld, debug: &DebugOverlays) {
    let obbs = &world.debug_snap().penetrating_obbs;
    if !debug.table_obb || obbs.is_empty() {
        for n in nodes.obbs.iter_mut() {
            n.set_visible(false).set_position(HIDDEN);
        }
        return;
    }
    for (i, n) in nodes.obbs.iter_mut().enumerate() {
        let Some(obb) = obbs.get(i) else {
            n.set_visible(false).set_position(HIDDEN);
            continue;
        };
        let ax = Vec3::new(
            obb.axes[0][0] as f32,
            obb.axes[0][1] as f32,
            obb.axes[0][2] as f32,
        );
        let ay = Vec3::new(
            obb.axes[1][0] as f32,
            obb.axes[1][1] as f32,
            obb.axes[1][2] as f32,
        );
        let az = Vec3::new(
            obb.axes[2][0] as f32,
            obb.axes[2][1] as f32,
            obb.axes[2][2] as f32,
        );
        // kiss3d cube local axes = world X/Y/Z; rotate so local Y → ay (primary)
        let rot = Quat::from_rotation_arc(Vec3::Y, ay.normalize_or_zero());
        let hx = (obb.half_extents[0] * 2.0) as f32;
        let hy = (obb.half_extents[1] * 2.0) as f32;
        let hz = (obb.half_extents[2] * 2.0) as f32;
        let _ = (ax, az);
        n.set_visible(true)
            .set_color(rgba(colors::OBB_HIT))
            .set_position(Vec3::new(
                obb.center[0] as f32,
                obb.center[1] as f32,
                obb.center[2] as f32,
            ))
            .set_rotation(rot)
            .set_local_scale(hx, hy, hz);
    }
}

fn sync_omega_arrow(nodes: &mut SceneDynamics, world: &SimWorld, debug: &DebugOverlays) {
    if !debug.omega_arrow {
        nodes.omega_shaft.set_visible(false).set_position(HIDDEN);
        nodes.omega_tip.set_visible(false).set_position(HIDDEN);
        return;
    }
    let w = world.debug_snap().omega;
    let dir = Vec3::new(w[0] as f32, w[1] as f32, w[2] as f32);
    let mag = dir.length();
    if mag < 1.0 {
        nodes.omega_shaft.set_visible(false).set_position(HIDDEN);
        nodes.omega_tip.set_visible(false).set_position(HIDDEN);
        return;
    }
    let origin = to_vec3(world.ball_position());
    let unit = dir / mag;
    let length = (0.08 + (mag * 0.002).min(0.25)) as f32;
    let tip_h = 0.035_f32;
    let shaft_h = (length - tip_h).max(0.02);
    let rot = Quat::from_rotation_arc(Vec3::Y, unit);
    nodes
        .omega_shaft
        .set_visible(true)
        .set_position(origin + unit * (shaft_h * 0.5))
        .set_rotation(rot)
        .set_local_scale(0.012, shaft_h, 0.012);
    nodes
        .omega_tip
        .set_visible(true)
        .set_position(origin + unit * (shaft_h + tip_h * 0.5))
        .set_rotation(rot);
}

fn sync_primitive_robot(nodes: &mut DynamicNodes, world: &SimWorld, debug: &DebugOverlays) {
    use crate::constants::geometry::{RACKET_BLADE_RADIUS, RACKET_HANDLE_RADIUS};

    let (rk_pos, rk_rot) = world.racket_pose();
    let blade = to_vec3(rk_pos);
    nodes
        .racket
        .set_position(blade)
        .set_rotation(racket_disc_world_rotation(rk_rot));

    let arm = world.arm();
    let joints = world.robot().joints();
    let Some(points) = arm.chain_points(world.robot().rail_x(), joints) else {
        return;
    };
    let points: Vec<Vec3> = points
        .into_iter()
        .map(|point| Vec3::new(point.x as f32, point.y as f32, point.z as f32))
        .collect();
    nodes.arm_base.set_position(points[0]);

    if points.len() >= 2 {
        let wrist = points[points.len() - 2];
        let to_blade = blade - wrist;
        let span = to_blade.length();
        let rim = if span > 1e-4 {
            let dir = to_blade / span;
            let inset = (RACKET_BLADE_RADIUS as f32).min(span * 0.95);
            blade - dir * inset
        } else {
            blade
        };
        place_link(
            &mut nodes.racket_handle,
            wrist,
            rim,
            RACKET_HANDLE_RADIUS as f32,
        );
    } else {
        nodes.racket_handle.set_position(HIDDEN);
    }

    let limits = &world.debug_snap().joint_at_limit;
    let limit_color = rgba(colors::JOINT_LIMIT);

    for (index, (link, joint)) in nodes
        .links
        .iter_mut()
        .zip(nodes.joints.iter_mut())
        .enumerate()
    {
        let Some((&from, &to)) = points.get(index).zip(points.get(index + 1)) else {
            link.set_position(HIDDEN);
            joint.set_position(HIDDEN);
            continue;
        };
        let at_limit = debug.joint_limits && limits.get(index).copied().unwrap_or(false);
        let j_color = if at_limit {
            limit_color
        } else {
            nodes.joint_color
        };
        let l_color = if at_limit {
            limit_color
        } else {
            nodes.link_color
        };
        joint.set_color(j_color);
        link.set_color(l_color);

        if index + 1 == points.len() - 1 {
            link.set_position(HIDDEN);
            joint.set_position(from);
            continue;
        }
        joint.set_position(to);
        let radius = nodes.link_radii.get(index).copied().unwrap_or(0.015);
        place_link(link, from, to, radius);
    }
}

fn sync_urdf_robot(
    nodes: &mut [UrdfVisualNode],
    urdf: &UrdfModel,
    joints: &[f64],
    mount: crate::robot::urdf::SimRobotMount,
    world: &SimWorld,
    debug: &DebugOverlays,
) {
    let poses: std::collections::HashMap<String, ([f64; 3], [f64; 4])> = urdf
        .link_poses_with_mount(joints, mount)
        .into_iter()
        .map(|(name, pos, quat)| (name, (pos, quat)))
        .collect();

    let any_limit = debug.joint_limits && world.debug_snap().joint_at_limit.iter().any(|&v| v);
    let limit_color = rgba(colors::JOINT_LIMIT);

    for entry in nodes.iter_mut() {
        let Some((link_pos, link_quat)) = poses.get(&entry.link_name) else {
            continue;
        };
        let link_tf = iso_from_pos_quat(*link_pos, *link_quat);
        let local_tf = iso_from_pos_quat(
            [
                entry.local_pos.x as f64,
                entry.local_pos.y as f64,
                entry.local_pos.z as f64,
            ],
            [
                entry.local_rot.w as f64,
                entry.local_rot.x as f64,
                entry.local_rot.y as f64,
                entry.local_rot.z as f64,
            ],
        );
        let world_tf = link_tf * local_tf;
        let t = world_tf.translation.vector;
        let q = world_tf.rotation.quaternion();
        entry
            .node
            .set_position(Vec3::new(t.x as f32, t.y as f32, t.z as f32))
            .set_rotation(Quat::from_xyzw(
                q.i as f32, q.j as f32, q.k as f32, q.w as f32,
            ));
        if any_limit {
            entry.node.set_color(limit_color);
        } else {
            entry.node.set_color(entry.base_color);
        }
    }
}

fn iso_from_pos_quat(pos: [f64; 3], quat_wxyz: [f64; 4]) -> nalgebra::Isometry3<f64> {
    use nalgebra::{Isometry3, Quaternion, UnitQuaternion, Vector3};
    let t = Vector3::new(pos[0], pos[1], pos[2]);
    let q = UnitQuaternion::new_normalize(Quaternion::new(
        quat_wxyz[0],
        quat_wxyz[1],
        quat_wxyz[2],
        quat_wxyz[3],
    ));
    return Isometry3::from_parts(t.into(), q);
}

fn rpy_xyz_to_pos(xyz: &[f64; 3]) -> Vec3 {
    return Vec3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
}

fn rpy_to_quat(rpy: [f64; 3]) -> Quat {
    let iso = iso_from_pos_quat([0.0, 0.0, 0.0], {
        let roll = rpy[0];
        let pitch = rpy[1];
        let yaw = rpy[2];
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    });
    let q = iso.rotation.quaternion();
    return Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32);
}

fn place_link(node: &mut SceneNode3d, from: Vec3, to: Vec3, radius: f32) {
    let dir = to - from;
    let length = dir.length().max(1e-4);
    let mid = (from + to) * 0.5;
    node.set_position(mid);
    if dir.length_squared() > 1e-8 {
        let axis = dir.normalize();
        let quat = Quat::from_rotation_arc(Vec3::Y, axis);
        node.set_rotation(quat);
    }
    let diameter = radius * 2.0;
    node.set_local_scale(diameter, length, diameter);
}

fn racket_disc_world_rotation(orientation: Rotation) -> Quat {
    let disc = Quat::from_rotation_arc(Vec3::Y, Vec3::Z);
    return to_quat(orientation) * disc;
}

fn to_vec3(v: Vector) -> Vec3 {
    return Vec3::new(v.x, v.y, v.z);
}

fn to_quat(r: Rotation) -> Quat {
    return Quat::from_xyzw(r.x, r.y, r.z, r.w);
}
