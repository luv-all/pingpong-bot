//! kiss3d 3D + egui — Rapier sim 월드와 슈터 패널 (macOS: 메인 스레드 단일 창).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::constants::{ball, table};
use kiss3d::prelude::*;
use rapier3d::prelude::{Rotation, Vector};
use tracing::info;

use super::controls::SimRuntimeControls;
use super::mesh_loader;
use super::panel;
use super::world::SimWorld;
use crate::robot::urdf::{UrdfLinkVisual, UrdfModel};

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
    racket: SceneNode3d,
    arm_base: SceneNode3d,
    links: Vec<SceneNode3d>,
    /// `links[i]` 반지름 [m]. `place_link`가 local_scale을 통째로 덮어쓰므로 보관.
    link_radii: Vec<f32>,
    joints: Vec<SceneNode3d>,
}

struct UrdfVisualNode {
    link_name: String,
    local_pos: Vec3,
    local_rot: Quat,
    node: SceneNode3d,
}

enum RobotRender {
    Primitive(DynamicNodes),
    Urdf(Vec<UrdfVisualNode>),
}

struct SceneDynamics {
    ball: SceneNode3d,
    shooter: SceneNode3d,
    robot: RobotRender,
    /// hit plane 예측 3D 위치 (공 비행 중 디버그)
    impact_marker: SceneNode3d,
    /// 접수 평면 위 투영 (x, y)
    impact_ring: SceneNode3d,
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
    camera.set_dist_step(1.04);
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
    ui_state.camera_dist = camera.dist();

    info!("kiss3d sim — 3D view + shooter panel (drag: orbit, scroll/slider: zoom)");

    let mut status_cache = None;
    while window.render_3d(&mut scene, &mut camera).await {
        if options.shutdown.load(Ordering::Acquire) {
            break;
        }
        if let Ok(snapshot) = options.world.try_lock() {
            sync_scene_dynamics(&mut dynamic, &snapshot, options.urdf.as_deref());
            status_cache = Some(panel::StatusSnapshot::from_world(&snapshot));
        }
        window.draw_ui(|ctx| {
            panel::draw(ctx, &mut ui_state, &controls, status_cache.as_ref());
        });
        camera.set_dist(ui_state.camera_dist);
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
}

fn build_scene_dynamics(scene: &mut SceneNode3d, urdf: Option<&UrdfModel>) -> SceneDynamics {
    let ball = scene
        .add_sphere(ball::RADIUS as f32)
        .set_color(Color::new(1.0, 0.55, 0.05, 1.0));
    let shooter = scene
        .add_cube(0.24, 0.5, 0.36)
        .set_color(Color::new(0.45, 0.45, 0.5, 1.0));
    let impact_marker = scene
        .add_sphere(0.018)
        .set_color(Color::new(1.0, 0.15, 0.95, 0.95));
    let impact_ring = scene
        .add_cube(0.05, 0.05, 0.004)
        .set_color(Color::new(1.0, 0.95, 0.1, 0.9));

    let robot = if let Some(model) = urdf {
        RobotRender::Urdf(build_urdf_nodes(scene, model))
    } else {
        RobotRender::Primitive(build_primitive_robot_nodes(scene))
    };

    return SceneDynamics {
        ball,
        shooter,
        robot,
        impact_marker,
        impact_ring,
    };
}

fn build_primitive_robot_nodes(scene: &mut SceneNode3d) -> DynamicNodes {
    use crate::constants::geometry::{
        ARM_BASE_HEIGHT, ARM_BASE_RADIUS, JOINT_MARKER_RADIUS, LINK_FOREARM_RADIUS,
        LINK_UPPER_RADIUS, RACKET_BLADE_RADIUS, RACKET_HALF_Z,
    };

    let link_color = Color::new(0.25, 0.45, 0.85, 1.0);
    let joint_color = Color::new(0.95, 0.85, 0.1, 1.0);
    // 상완·전완 반경을 번갈아 쓴다 (체인 세그먼트 0=베이스→q0는 상완 쪽).
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
        arm_base: scene
            .add_cylinder(ARM_BASE_RADIUS as f32, ARM_BASE_HEIGHT as f32)
            .set_color(Color::new(0.2, 0.25, 0.55, 1.0)),
        links,
        link_radii: link_radii.iter().map(|&r| r as f32).collect(),
        joints,
    };
}

fn build_urdf_nodes(scene: &mut SceneNode3d, urdf: &UrdfModel) -> Vec<UrdfVisualNode> {
    return urdf
        .link_visuals()
        .into_iter()
        .map(|vis| {
            let link_name = vis.link_name.clone();
            UrdfVisualNode {
                link_name,
                local_pos: rpy_xyz_to_pos(&vis.origin_xyz),
                local_rot: rpy_to_quat(vis.origin_rpy),
                node: add_urdf_visual(scene, &vis),
            }
        })
        .collect();
}

fn add_urdf_visual(scene: &mut SceneNode3d, vis: &UrdfLinkVisual) -> SceneNode3d {
    let color = Color::new(vis.color[0], vis.color[1], vis.color[2], vis.color[3]);
    return mesh_loader::add_geometry(scene, &vis.geometry, color);
}

fn sync_scene_dynamics(nodes: &mut SceneDynamics, world: &SimWorld, urdf: Option<&UrdfModel>) {
    let ball = world.ball_position();
    nodes.ball.set_position(to_vec3(ball));

    let (sh_pos, sh_rot) = world.shooter_pose();
    nodes
        .shooter
        .set_position(to_vec3(sh_pos))
        .set_rotation(to_quat(sh_rot));

    sync_impact_debug_markers(nodes, world);

    match (&mut nodes.robot, urdf) {
        (RobotRender::Primitive(arm_nodes), None) => {
            sync_primitive_robot(arm_nodes, world);
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
            );
        }
        _ => {}
    }
}

fn sync_impact_debug_markers(nodes: &mut SceneDynamics, world: &SimWorld) {
    const HIDDEN: Vec3 = Vec3::new(0.0, 0.0, -10.0);
    let Some(pred) = world.debug_prediction() else {
        nodes.impact_marker.set_position(HIDDEN);
        nodes.impact_ring.set_position(HIDDEN);
        return;
    };
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

fn sync_primitive_robot(nodes: &mut DynamicNodes, world: &SimWorld) {
    let (rk_pos, rk_rot) = world.racket_pose();
    nodes
        .racket
        .set_position(to_vec3(rk_pos))
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

    const HIDDEN: Vec3 = Vec3::new(0.0, 0.0, -10.0);
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
) {
    let poses: std::collections::HashMap<String, ([f64; 3], [f64; 4])> = urdf
        .link_poses_with_mount(joints, mount)
        .into_iter()
        .map(|(name, pos, quat)| (name, (pos, quat)))
        .collect();

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
    // kiss3d unit cylinder는 local_scale=(diameter, height, diameter).
    // XZ를 1로 두면 지름 1 m 원반이 된다.
    let diameter = radius * 2.0;
    node.set_local_scale(diameter, length, diameter);
}

fn racket_disc_world_rotation(orientation: Rotation) -> Quat {
    // kiss3d 실린더 축은 Y. 라켓 계약은 local +Z = 면 법선.
    let disc = Quat::from_rotation_arc(Vec3::Y, Vec3::Z);
    return to_quat(orientation) * disc;
}

fn to_vec3(v: Vector) -> Vec3 {
    return Vec3::new(v.x, v.y, v.z);
}

fn to_quat(r: Rotation) -> Quat {
    return Quat::from_xyzw(r.x, r.y, r.z, r.w);
}
