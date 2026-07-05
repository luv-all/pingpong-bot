//! kiss3d 3D + egui — Rapier sim 월드와 슈터 패널 (macOS: 메인 스레드 단일 창).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use kiss3d::prelude::*;
use pingpong_domain::constants::{ball, table};
use pingpong_domain::{Arm, Joints};
use rapier3d::prelude::{Rotation, Vector};
use tracing::info;

use super::controls::SimRuntimeControls;
use super::mesh_loader;
use super::panel;
use super::world::SimWorld;
use crate::urdf::{UrdfLinkVisual, UrdfRobot};

/// sim 3D + 제어 패널 옵션.
pub struct SimViewerOptions {
    /// 발사·sim 설정
    pub controls: Arc<Mutex<SimRuntimeControls>>,
    /// 공유 sim 월드
    pub world: Arc<Mutex<SimWorld>>,
    /// URDF 모델 (kiss3d 로봇 mesh 대신 사용)
    pub urdf: Option<Arc<crate::urdf::UrdfRobot>>,
    /// 창 닫을 때 파이프라인 종료
    pub shutdown: Arc<AtomicBool>,
}

/// kiss3d 창을 메인 스레드에서 연다 (블로킹).
pub fn run(options: SimViewerOptions) -> Result<(), String> {
    return pollster::block_on(viewer_main(options));
}

struct DynamicNodes {
    ball: SceneNode3d,
    shooter: SceneNode3d,
    racket: SceneNode3d,
    arm_base: SceneNode3d,
    links: [SceneNode3d; 3],
    joints: [SceneNode3d; 3],
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
    let mut ui_state = panel::PanelUiState::from_controls(
        &options.controls.lock().expect("controls"),
    );
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
            panel::draw(
                ctx,
                &mut ui_state,
                &controls,
                status_cache.as_ref(),
            );
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
}

fn build_scene_dynamics(scene: &mut SceneNode3d, urdf: Option<&UrdfRobot>) -> SceneDynamics {
    let ball = scene
        .add_sphere(ball::RADIUS as f32)
        .set_color(Color::new(1.0, 0.55, 0.05, 1.0));
    let shooter = scene
        .add_cube(0.24, 0.5, 0.36)
        .set_color(Color::new(0.45, 0.45, 0.5, 1.0));

    let robot = if let Some(model) = urdf {
        RobotRender::Urdf(build_urdf_nodes(scene, model))
    } else {
        RobotRender::Primitive(build_primitive_robot_nodes(scene))
    };

    return SceneDynamics { ball, shooter, robot };
}

fn build_primitive_robot_nodes(scene: &mut SceneNode3d) -> DynamicNodes {
    let link_color = Color::new(0.25, 0.45, 0.85, 1.0);
    let joint_color = Color::new(0.95, 0.85, 0.1, 1.0);

    return DynamicNodes {
        ball: scene
            .add_sphere(ball::RADIUS as f32)
            .set_color(Color::new(1.0, 0.55, 0.05, 1.0)),
        shooter: scene
            .add_cube(0.24, 0.5, 0.36)
            .set_color(Color::new(0.45, 0.45, 0.5, 1.0)),
        racket: scene
            .add_cube(0.16, 0.18, 0.012)
            .set_color(Color::new(0.85, 0.15, 0.12, 1.0)),
        arm_base: scene
            .add_cylinder(0.12, 0.08)
            .set_color(Color::new(0.2, 0.25, 0.55, 1.0)),
        links: [
            scene.add_cylinder(0.045, 0.35).set_color(link_color),
            scene.add_cylinder(0.04, 0.30).set_color(link_color),
            scene.add_cylinder(0.035, 0.15).set_color(link_color),
        ],
        joints: [
            scene.add_sphere(0.05).set_color(joint_color),
            scene.add_sphere(0.045).set_color(joint_color),
            scene.add_sphere(0.04).set_color(joint_color),
        ],
    };
}

fn build_urdf_nodes(scene: &mut SceneNode3d, urdf: &UrdfRobot) -> Vec<UrdfVisualNode> {
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

fn sync_scene_dynamics(nodes: &mut SceneDynamics, world: &SimWorld, urdf: Option<&UrdfRobot>) {
    let ball = world.ball_position();
    nodes.ball.set_position(to_vec3(ball));

    let (sh_pos, sh_rot) = world.shooter_pose();
    nodes
        .shooter
        .set_position(to_vec3(sh_pos))
        .set_rotation(to_quat(sh_rot));

    match (&mut nodes.robot, urdf) {
        (RobotRender::Primitive(arm_nodes), None) => {
            sync_primitive_robot(arm_nodes, world);
        }
        (RobotRender::Urdf(urdf_nodes), Some(model)) => {
            sync_urdf_robot(
                urdf_nodes,
                model,
                world.robot().joints().values.as_slice(),
            );
        }
        _ => {}
    }
}

fn sync_primitive_robot(nodes: &mut DynamicNodes, world: &SimWorld) {
    let ball = world.ball_position();
    nodes.ball.set_position(to_vec3(ball));

    let (sh_pos, sh_rot) = world.shooter_pose();
    nodes
        .shooter
        .set_position(to_vec3(sh_pos))
        .set_rotation(to_quat(sh_rot));

    let (rk_pos, rk_rot) = world.racket_pose();
    nodes
        .racket
        .set_position(to_vec3(rk_pos))
        .set_rotation(to_quat(rk_rot));

    let arm = world.arm();
    let joints = world.robot().joints();
    let points = arm_chain_points(arm, joints);
    nodes.arm_base.set_position(points[0]);

    let lengths = [
        arm.link_lengths[0] as f32,
        arm.link_lengths[1] as f32,
        arm.link_lengths[2] as f32,
    ];
    for i in 0..3 {
        nodes.joints[i].set_position(points[i + 1]);
        place_link(&mut nodes.links[i], points[i], points[i + 1], lengths[i]);
    }
}

fn sync_urdf_robot(nodes: &mut [UrdfVisualNode], urdf: &UrdfRobot, joints: &[f64]) {
    let poses: std::collections::HashMap<String, ([f64; 3], [f64; 4])> = urdf
        .link_poses_in_sim(joints)
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
            .set_rotation(Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32));
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

fn arm_chain_points(arm: &Arm, joints: &Joints) -> [Vec3; 4] {
    let yaw = joints.values[0] as f32;
    let a1 = joints.values[1] as f32;
    let a2 = joints.values[2] as f32;
    let elbow = a1 + a2;
    let l1 = arm.link_lengths[0] as f32;
    let l2 = arm.link_lengths[1] as f32;
    let l3 = arm.link_lengths[2] as f32;

    let base = Vec3::new(
        arm.base.v.x as f32,
        arm.base.v.y as f32,
        arm.base.v.z as f32,
    );

    let to_world = |reach: f32, height: f32| -> Vec3 {
        return base + Vec3::new(reach * yaw.sin(), reach * yaw.cos(), height);
    };

    return [
        base,
        to_world(l1 * a1.cos(), l1 * a1.sin()),
        to_world(
            l1 * a1.cos() + l2 * elbow.cos(),
            l1 * a1.sin() + l2 * elbow.sin(),
        ),
        to_world(
            l1 * a1.cos() + l2 * elbow.cos() + l3 * elbow.cos(),
            l1 * a1.sin() + l2 * elbow.sin() + l3 * elbow.sin(),
        ),
    ];
}

fn place_link(node: &mut SceneNode3d, from: Vec3, to: Vec3, length: f32) {
    let dir = to - from;
    let mid = (from + to) * 0.5;
    node.set_position(mid);
    if dir.length_squared() > 1e-8 {
        let axis = dir.normalize();
        let quat = Quat::from_rotation_arc(Vec3::Y, axis);
        node.set_rotation(quat);
    }
    let _ = length;
}

fn to_vec3(v: Vector) -> Vec3 {
    return Vec3::new(v.x, v.y, v.z);
}

fn to_quat(r: Rotation) -> Quat {
    return Quat::from_xyzw(r.x, r.y, r.z, r.w);
}
