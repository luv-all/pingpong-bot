//! 재사용 kiss3d 탁구대·공 씬 (`feature = "gui"`).
//!
//! 메인 [`super::viewer`]·[`super::host`]·후속 툴(verify-stereo, interactive jog)이
//! 같은 월드 비주얼을 쓴다.
//!
//! **레이어 R/W**는 [`super::layers`]. 호스트 조립은 [`super::host`].

use kiss3d::prelude::*;

use crate::constants::table;

/// 화면 밖 숨김 위치 (마커·공 공통 패턴).
pub const HIDDEN: Vec3 = Vec3::new(0.0, 0.0, -10.0);

/// [`build_table_scene`] 옵션. 기본값은 메인 sim 뷰어와 동일(레일·축 포함).
#[derive(Clone, Debug)]
pub struct TableSceneOptions {
    /// 레일 프레임 시각화 (`defaults::rail_frame` 위치).
    pub include_rail: bool,
    /// 월드 원점(테이블 로봇쪽 코너) XYZ 화살표.
    pub include_axes: bool,
}

impl Default for TableSceneOptions {
    fn default() -> Self {
        return Self {
            include_rail: true,
            include_axes: true,
        };
    }
}

/// 탁구대·네트·바닥·(옵션) 레일·축을 씬에 추가한다.
///
/// 치수는 `constants` SSOT만 사용한다. 레일 프로파일 노드는 초기 위치를
/// `defaults::rail_frame`으로 잡되 **핸들을 돌려준다** — 마운트는 런타임에
/// 움직일 수 있으므로(`SimRuntimeControls::rail_frame`) 라이브 뷰어가 매 프레임
/// `arm.rail` 기준으로 다시 배치해야 한다. 정적 씬만 필요한 호출자는 버리면 된다.
pub(crate) fn build_table_scene(
    scene: &mut SceneNode3d,
    opts: &TableSceneOptions,
) -> Option<SceneNode3d> {
    let tw = table::WIDTH_X as f32;
    let tl = table::LENGTH_Y as f32;
    let tcx = tw * 0.5;
    let tcy = tl * 0.5;
    let thick = table::HALF_THICKNESS as f32 * 2.0;
    let surface_z = table::SURFACE_Z as f32;
    let table_z = surface_z - thick * 0.5;

    // 상판 — ITTF 블루에 가까운 딥 블루그린
    let top = Color::new(0.05, 0.32, 0.48, 1.0);
    let apron = Color::new(0.08, 0.10, 0.12, 1.0);
    let line = Color::new(0.96, 0.96, 0.94, 1.0);
    let metal = Color::new(0.42, 0.44, 0.47, 1.0);
    let foot = Color::new(0.18, 0.18, 0.20, 1.0);

    scene
        .add_cube(tw, tl, thick)
        .set_color(top)
        .set_position(Vec3::new(tcx, tcy, table_z));

    // 상판 아래 에이프런(가장자리 띠)
    let apron_h = 0.04_f32;
    let apron_t = 0.018_f32;
    let apron_z = surface_z - thick - apron_h * 0.5;
    scene
        .add_cube(tw, apron_t, apron_h)
        .set_color(apron)
        .set_position(Vec3::new(tcx, apron_t * 0.5, apron_z));
    scene
        .add_cube(tw, apron_t, apron_h)
        .set_color(apron)
        .set_position(Vec3::new(tcx, tl - apron_t * 0.5, apron_z));
    scene
        .add_cube(apron_t, tl, apron_h)
        .set_color(apron)
        .set_position(Vec3::new(apron_t * 0.5, tcy, apron_z));
    scene
        .add_cube(apron_t, tl, apron_h)
        .set_color(apron)
        .set_position(Vec3::new(tw - apron_t * 0.5, tcy, apron_z));

    add_table_court_lines(scene, tw, tl, tcx, tcy, surface_z, line);
    add_table_legs(scene, tw, tl, surface_z - thick, metal, foot);
    add_net_cloth(scene, tcx, tcy);

    scene
        .add_cube(tw * 1.2, tl * 1.2, 0.02)
        .set_color(Color::new(0.22, 0.23, 0.25, 1.0))
        .set_position(Vec3::new(tcx, tcy, 0.01));

    let rail = opts.include_rail.then(|| {
        let frame = crate::defaults::rail_frame();
        let rail_w = crate::constants::geometry::RAIL_VISUAL_WIDTH as f32;
        let mut node = scene.add_cube(tw, rail_w, rail_profile_thickness());
        node.set_color(Color::new(0.35, 0.38, 0.42, 1.0))
            .set_position(rail_profile_center(
                frame.mount_x() as f32 + crate::defaults::RAIL_PHYSICAL_X_MAX_M as f32 * 0.5,
                frame.mount_y(),
                frame.mount_z(),
            ));
        return node;
    });

    if opts.include_axes {
        // 테이블 로봇쪽 코너 윗면 = 월드 (0, 0, SURFACE_Z)
        let axis_origin = Vec3::new(0.0, 0.0, surface_z);
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

    return rail;
}

/// 레일 프로파일 큐브 높이 [m] — 실측 두께.
pub(crate) fn rail_profile_thickness() -> f32 {
    return crate::constants::geometry::RAIL_THICKNESS as f32;
}

/// 마운트(y, z)에 대한 레일 프로파일 큐브 중심.
///
/// 베이스는 프로파일 **윗면**에 얹히므로 `mount_z`가 윗면이고, 큐브 중심은
/// 두께 절반만큼 아래다.
pub(crate) fn rail_profile_center(table_center_x: f32, mount_y: f64, mount_z: f64) -> Vec3 {
    return Vec3::new(
        table_center_x,
        mount_y as f32,
        mount_z as f32 - rail_profile_thickness() * 0.5,
    );
}

/// ITTF식 백선: 외곽(~20mm) + 중앙 세로선(~3mm). 살짝 띄워 z-fighting 방지.
fn add_table_court_lines(
    scene: &mut SceneNode3d,
    tw: f32,
    tl: f32,
    tcx: f32,
    tcy: f32,
    surface_z: f32,
    line: Color,
) {
    let z = surface_z + 0.0015;
    let border = 0.020_f32;
    let center_w = 0.004_f32;
    let h = 0.0012_f32;

    scene
        .add_cube(tw, border, h)
        .set_color(line)
        .set_position(Vec3::new(tcx, border * 0.5, z));
    scene
        .add_cube(tw, border, h)
        .set_color(line)
        .set_position(Vec3::new(tcx, tl - border * 0.5, z));
    let side_len = (tl - 2.0 * border).max(0.01);
    scene
        .add_cube(border, side_len, h)
        .set_color(line)
        .set_position(Vec3::new(border * 0.5, tcy, z));
    scene
        .add_cube(border, side_len, h)
        .set_color(line)
        .set_position(Vec3::new(tw - border * 0.5, tcy, z));
    scene
        .add_cube(center_w, tl - 2.0 * border, h)
        .set_color(line)
        .set_position(Vec3::new(tcx, tcy, z));
}

fn add_table_legs(
    scene: &mut SceneNode3d,
    tw: f32,
    tl: f32,
    top_underside_z: f32,
    metal: Color,
    foot: Color,
) {
    let inset = 0.12_f32;
    let leg_w = 0.045_f32;
    let foot_h = 0.025_f32;
    let leg_h = (top_underside_z - foot_h).max(0.05);
    let leg_z = foot_h + leg_h * 0.5;
    let xs = [inset, tw - inset];
    let ys = [inset, tl - inset];

    for &x in &xs {
        for &y in &ys {
            scene
                .add_cube(leg_w, leg_w, leg_h)
                .set_color(metal)
                .set_position(Vec3::new(x, y, leg_z));
            scene
                .add_cube(leg_w * 1.6, leg_w * 1.6, foot_h)
                .set_color(foot)
                .set_position(Vec3::new(x, y, foot_h * 0.5));
        }
    }

    let brace_z = foot_h + leg_h * 0.28;
    let brace_t = 0.022_f32;
    for &y in &ys {
        scene
            .add_cube(tw - 2.0 * inset, brace_t, brace_t)
            .set_color(metal)
            .set_position(Vec3::new(tw * 0.5, y, brace_z));
    }
    for &x in &xs {
        scene
            .add_cube(brace_t, tl - 2.0 * inset, brace_t)
            .set_color(metal)
            .set_position(Vec3::new(x, tl * 0.5, brace_z));
    }
}

/// 네트 외관 — 격자 cloth 메쉬 (물리 soft body 아님; Rapier는 soft 실체 판).
fn add_net_cloth(scene: &mut SceneNode3d, table_cx: f32, table_cy: f32) {
    let net_h = table::NET_HEIGHT as f32;
    let net_w = table::WIDTH_X as f32;
    let z0 = table::SURFACE_Z as f32;
    let cord = Color::new(0.12, 0.28, 0.18, 0.88);
    let post = Color::new(0.10, 0.10, 0.11, 1.0);
    let tape = Color::new(0.94, 0.94, 0.92, 0.98);

    let post_t = 0.014_f32;
    for x in [0.0_f32, net_w] {
        scene
            .add_cube(post_t, post_t, net_h + 0.02)
            .set_color(post)
            .set_position(Vec3::new(x, table_cy, z0 + net_h * 0.5));
    }
    scene
        .add_cube(net_w, 0.008, 0.014)
        .set_color(tape)
        .set_position(Vec3::new(table_cx, table_cy, z0 + net_h - 0.007));

    const NX: usize = 22;
    const NZ: usize = 8;
    let cord_t = 0.0025_f32;
    for i in 0..=NX {
        let x = net_w * (i as f32) / (NX as f32);
        scene
            .add_cube(cord_t, cord_t, net_h - 0.014)
            .set_color(cord)
            .set_position(Vec3::new(x, table_cy, z0 + (net_h - 0.014) * 0.5));
    }
    for j in 1..NZ {
        let z = z0 + net_h * (j as f32) / (NZ as f32);
        scene
            .add_cube(net_w, cord_t, cord_t)
            .set_color(cord)
            .set_position(Vec3::new(table_cx, table_cy, z));
    }
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
