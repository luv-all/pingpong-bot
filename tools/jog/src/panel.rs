//! jog egui 패널.

use kiss3d::egui::{self, Color32, RichText};
use pingpong_bot::InterceptWindow;
use pingpong_bot::constants::table;

use crate::motion::{MotionKind, REACH_DELTA_M, joint_label, reach_ok};
use crate::state::{Action, JogApp, try_action};

pub fn draw(ctx: &egui::Context, app: &mut JogApp) {
    ensure_korean_fonts(ctx);

    egui::Window::new("Jog")
        .default_pos(egui::pos2(12.0, 12.0))
        .default_width(400.0)
        .resizable(true)
        .show(ctx, |ui| {
            app.sync_arrival_ghost();
            draw_header(ui, app);
            ui.separator();
            draw_status(ui, app);
            ui.separator();
            draw_params(ui, app);
            ui.separator();
            draw_motion(ui, app);
            ui.separator();
            draw_actions(ui, app);
            if let Some(err) = &app.error {
                ui.add_space(4.0);
                ui.colored_label(Color32::from_rgb(220, 90, 80), err);
            }
        });
}

fn draw_header(ui: &mut egui::Ui, app: &JogApp) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(app.phase.label()).strong());
        if app.dry_run {
            ui.colored_label(Color32::from_rgb(180, 160, 60), "dry-run");
        }
        if app.sim_busy() {
            ui.colored_label(Color32::from_rgb(90, 160, 220), "시뮬 재생 중");
        }
    });
}

fn draw_status(ui: &mut egui::Ui, app: &JogApp) {
    let Some(pose) = app.live_pose() else {
        ui.label("로봇: —");
        return;
    };
    ui.label(format!("레일 = {:.4} m", pose.rail_x));
    for (i, rad) in pose.joints.values.iter().enumerate() {
        ui.label(format!("{} = {:.1}°", joint_label(i), rad.to_degrees()));
    }
    if let Some(fk) = app
        .arm
        .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
    {
        ui.label(format!(
            "라켓 = ({:.3}, {:.3}, {:.3}) m",
            fk.position.coords.x, fk.position.coords.y, fk.position.coords.z
        ));
    }
}

fn draw_params(ui: &mut egui::Ui, app: &mut JogApp) {
    ui.label(RichText::new("이동 설정").strong());
    ranged(
        ui,
        "이동 시간 [초]",
        &mut app.duration_secs,
        0.05,
        10.0,
        0.05,
    );
    ranged(
        ui,
        "한 번에 최대 각도 [°]",
        &mut app.max_delta_deg,
        1.0,
        90.0,
        0.5,
    );
}

fn draw_motion(ui: &mut egui::Ui, app: &mut JogApp) {
    ui.label(RichText::new("모션").strong());
    egui::ComboBox::from_id_salt("motion_kind")
        .selected_text(app.draft.kind.label())
        .width(ui.available_width().min(320.0))
        .show_ui(ui, |ui| {
            for kind in [
                MotionKind::Joint,
                MotionKind::Angles,
                MotionKind::RailAbs,
                MotionKind::Ik,
                MotionKind::Pose,
                MotionKind::Swing,
                MotionKind::AimBall,
                MotionKind::SwingBall,
            ] {
                ui.selectable_value(&mut app.draft.kind, kind, kind.label());
            }
        });

    let (rail_min, rail_max) = rail_range(&app.arm);

    match app.draft.kind {
        MotionKind::Joint => {
            ui.horizontal(|ui| {
                ui.label("관절");
                egui::ComboBox::from_id_salt("joint_pick")
                    .selected_text(joint_label(app.draft.joint_index))
                    .show_ui(ui, |ui| {
                        for i in 0..4 {
                            ui.selectable_value(&mut app.draft.joint_index, i, joint_label(i));
                        }
                    });
            });
            let (min, max) = joint_jog_deg_range(
                &app.arm,
                app.draft.joint_index,
                app.synced_pose.as_ref(),
                app.max_delta_deg,
            );
            ranged(
                ui,
                &format!("{} [°]", joint_label(app.draft.joint_index)),
                &mut app.draft.joint_deg,
                min,
                max,
                0.5,
            );
        }
        MotionKind::Angles => {
            for i in 0..app.draft.angles_deg.len() {
                let (min, max) =
                    joint_jog_deg_range(&app.arm, i, app.synced_pose.as_ref(), app.max_delta_deg);
                ranged(
                    ui,
                    &format!("{} [°]", joint_label(i)),
                    &mut app.draft.angles_deg[i],
                    min,
                    max,
                    0.5,
                );
            }
        }
        MotionKind::RailAbs => {
            ranged(
                ui,
                "레일 위치 [m]",
                &mut app.draft.rail_x,
                rail_min,
                rail_max,
                0.005,
            );
        }
        MotionKind::Ik => {
            draw_reach(ui, app, false);
        }
        MotionKind::Pose => {
            draw_reach(ui, app, true);
        }
        MotionKind::Swing => {
            draw_reach(ui, app, true);
            ranged(
                ui,
                "맞을 때 속도 [m/s]",
                &mut app.draft.swing_speed,
                0.1,
                8.0,
                0.05,
            );
        }
        MotionKind::AimBall => {
            draw_arrival(ui, app, false);
        }
        MotionKind::SwingBall => {
            draw_arrival(ui, app, true);
        }
    }
}

fn draw_arrival(ui: &mut egui::Ui, app: &mut JogApp, with_ball_vel: bool) {
    ui.label("공 도달점 [m]");
    ranged(
        ui,
        "x",
        &mut app.draft.arrival_xyz[0],
        -0.05,
        table::WIDTH_X + 0.05,
        0.005,
    );
    // 도달점 = 접수 창(hit plane). InterceptWindow 기본 [0.08, 0.35].
    let hit = InterceptWindow::default();
    ranged(
        ui,
        "y",
        &mut app.draft.arrival_xyz[1],
        hit.y_min,
        hit.y_max,
        0.005,
    );
    ranged(
        ui,
        "z",
        &mut app.draft.arrival_xyz[2],
        table::SURFACE_Z,
        table::SURFACE_Z + 0.6,
        0.005,
    );

    ui.label("면 기울기 [°]");
    ranged(ui, "pitch", &mut app.draft.tilt_pitch_deg, -30.0, 30.0, 0.5);
    ranged(ui, "yaw", &mut app.draft.tilt_yaw_deg, -30.0, 30.0, 0.5);

    if with_ball_vel {
        ui.label("공 입사 속도 [m/s]");
        ranged(ui, "vx", &mut app.draft.ball_vin[0], -8.0, 8.0, 0.05);
        ranged(ui, "vy", &mut app.draft.ball_vin[1], -12.0, 2.0, 0.05);
        ranged(ui, "vz", &mut app.draft.ball_vin[2], -8.0, 4.0, 0.05);
    }

    if let Some(pose) = app.synced_pose.as_ref() {
        let ok = reach_ok(&app.arm, pose, &app.draft);
        if ok {
            ui.colored_label(Color32::from_rgb(90, 190, 120), "IK 가능");
        } else {
            ui.colored_label(Color32::from_rgb(220, 90, 80), "IK 불가");
        }
    }
}

fn draw_reach(ui: &mut egui::Ui, app: &mut JogApp, with_tilt: bool) {
    let d = REACH_DELTA_M;
    if let Some(pose) = app.synced_pose.as_ref() {
        if let Some(fk) = app
            .arm
            .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
        {
            ui.label(format!(
                "지금 라켓 ({:.3}, {:.3}, {:.3})",
                fk.position.coords.x, fk.position.coords.y, fk.position.coords.z
            ));
        }
    } else {
        ui.label("동기화 후 현재 라켓 기준으로 옮깁니다");
    }

    ui.label("이동 [m]");
    ranged(ui, "Δx", &mut app.draft.reach_dxyz[0], -d, d, 0.005);
    ranged(ui, "Δy", &mut app.draft.reach_dxyz[1], -d, d, 0.005);
    ranged(ui, "Δz", &mut app.draft.reach_dxyz[2], -d, d, 0.005);

    if with_tilt {
        ui.label("기울기 [°]");
        ranged(ui, "pitch", &mut app.draft.tilt_pitch_deg, -30.0, 30.0, 0.5);
        ranged(ui, "yaw", &mut app.draft.tilt_yaw_deg, -30.0, 30.0, 0.5);
    }

    if let Some(pose) = app.synced_pose.as_ref() {
        let ok = reach_ok(&app.arm, pose, &app.draft);
        if ok {
            ui.colored_label(Color32::from_rgb(90, 190, 120), "IK 가능");
        } else {
            ui.colored_label(Color32::from_rgb(220, 90, 80), "IK 불가 — Δ를 줄이세요");
        }
    }
}

fn draw_actions(ui: &mut egui::Ui, app: &mut JogApp) {
    ui.horizontal(|ui| {
        if ui
            .add_enabled(app.phase.can_preview(), egui::Button::new("미리보기"))
            .clicked()
        {
            try_action(app, Action::Preview);
        }
        if ui
            .add_enabled(app.phase.can_discard(), egui::Button::new("버리기"))
            .clicked()
        {
            try_action(app, Action::Discard);
        }
        if ui
            .add_enabled(app.phase.can_apply(), egui::Button::new("적용"))
            .clicked()
        {
            try_action(app, Action::Apply);
        }
        if ui
            .add_enabled(app.phase.can_sync(), egui::Button::new("동기화"))
            .clicked()
        {
            try_action(app, Action::Sync);
        }
    });
}

/// 슬라이더 + 숫자 입력. 아래에 한계 표시.
fn ranged(ui: &mut egui::Ui, label: &str, value: &mut f64, min: f64, max: f64, speed: f64) {
    let (lo, hi) = if min < max { (min, max) } else { (max, min) };
    *value = value.clamp(lo, hi);
    ui.label(label);
    ui.horizontal(|ui| {
        ui.add(
            egui::Slider::new(value, lo..=hi)
                .show_value(false)
                .clamping(egui::SliderClamping::Always),
        );
        ui.add(
            egui::DragValue::new(value)
                .speed(speed)
                .range(lo..=hi)
                .max_decimals(3),
        );
    });
    ui.label(
        RichText::new(format!("{}  ~  {}", format_bound(lo), format_bound(hi)))
            .weak()
            .small(),
    );
}

fn joint_hw_deg_range(arm: &pingpong_bot::Arm, index: usize) -> (f64, f64) {
    if let Some(limit) = arm.joint_limit(index) {
        return (limit.min.to_degrees(), limit.max.to_degrees());
    }
    return (-180.0, 180.0);
}

/// 관절 한계 ∩ (현재각 ± maxdelta). 슬라이더가 maxdelta 위반을 미리 막는다.
fn joint_jog_deg_range(
    arm: &pingpong_bot::Arm,
    index: usize,
    synced: Option<&pingpong_bot::RobotPose>,
    max_delta_deg: f64,
) -> (f64, f64) {
    let (hw_lo, hw_hi) = joint_hw_deg_range(arm, index);
    let Some(pose) = synced else {
        return (hw_lo, hw_hi);
    };
    let Some(cur) = pose.joints.values.get(index) else {
        return (hw_lo, hw_hi);
    };
    let cur_deg = cur.to_degrees();
    let delta = max_delta_deg.max(0.0);
    let lo = hw_lo.max(cur_deg - delta);
    let hi = hw_hi.min(cur_deg + delta);
    if lo <= hi {
        return (lo, hi);
    }
    return (cur_deg, cur_deg);
}

fn rail_range(arm: &pingpong_bot::Arm) -> (f64, f64) {
    if let Some(rail) = arm.rail {
        return (rail.x_min, rail.x_max);
    }
    return (0.0, 1.41);
}

fn format_bound(v: f64) -> String {
    if v.fract().abs() < 1e-6 {
        return format!("{v:.0}");
    }
    if v.abs() >= 10.0 {
        return format!("{v:.1}");
    }
    return format!("{v:.2}");
}

fn ensure_korean_fonts(ctx: &egui::Context) {
    use std::sync::atomic::{AtomicBool, Ordering};
    static INSTALLED: AtomicBool = AtomicBool::new(false);
    if INSTALLED.swap(true, Ordering::Relaxed) {
        return;
    }
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "NanumGothic".to_owned(),
        egui::FontData::from_static(include_bytes!(
            "../../../assets/fonts/NanumGothic-Regular.ttf"
        ))
        .into(),
    );
    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .push("NanumGothic".to_owned());
    fonts
        .families
        .entry(egui::FontFamily::Monospace)
        .or_default()
        .push("NanumGothic".to_owned());
    ctx.set_fonts(fonts);
}
