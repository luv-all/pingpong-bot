//! jog egui 패널.

use kiss3d::egui::{self, Color32, RichText};
use pingpong_bot::robot::motion::InterceptWindow;
use pingpong_bot::sim::gui::shooter;

use crate::plan::{Kind, REACH_DELTA_M, SwingPreview, joint_label, reach_ok, swing_preview};
use crate::state::{Action, JogApp, try_action};

pub fn draw(ctx: &egui::Context, app: &mut JogApp) {
    ensure_korean_fonts(ctx);

    // 예측은 프레임당 한 번만 — 고스트 공·표시·미리보기 게이트가 같은 값을 쓴다.
    let preview = if app.draft.kind == Kind::Swing {
        app.synced_pose
            .as_ref()
            .and_then(|pose| swing_preview(&app.arm, pose, &app.draft).ok())
    } else {
        None
    };
    app.sync_ball_ghost(preview.as_ref());

    draw_shooter_window(ctx, app);

    egui::Window::new("Jog")
        .default_pos(egui::pos2(12.0, 12.0))
        .default_width(400.0)
        .resizable(true)
        .show(ctx, |ui| {
            draw_header(ui, app);
            ui.separator();
            draw_status(ui, app);
            ui.separator();
            draw_params(ui, app);
            ui.separator();
            draw_motion(ui, app, preview.as_ref());
            ui.separator();
            draw_actions(ui, app, preview.as_ref());
            if let Some(err) = &app.error {
                ui.add_space(4.0);
                ui.colored_label(Color32::from_rgb(220, 90, 80), err);
            }
        });
}

/// 메인 sim과 같은 슈터 위젯. 값이 바뀌면 곧바로 sim controls로 민다.
fn draw_shooter_window(ctx: &egui::Context, app: &mut JogApp) {
    egui::Window::new("슈터")
        .default_pos(egui::pos2(440.0, 12.0))
        .default_width(280.0)
        .resizable(true)
        .show(ctx, |ui| {
            let buttons = shooter::ui::draw(ui, &mut app.draft.shooter);
            if buttons.random {
                app.draft.shooter = app.draft.shooter.randomized(&mut rand::thread_rng());
            }
            app.push_shooter();
            if let Some(handle) = &app.shooter {
                if buttons.shoot {
                    handle.request_shoot();
                }
                if buttons.park {
                    handle.request_park();
                }
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

fn draw_motion(ui: &mut egui::Ui, app: &mut JogApp, preview: Option<&SwingPreview>) {
    ui.label(RichText::new("모션").strong());
    egui::ComboBox::from_id_salt("motion_kind")
        .selected_text(app.draft.kind.label())
        .width(ui.available_width().min(320.0))
        .show_ui(ui, |ui| {
            for kind in [
                Kind::Joint,
                Kind::Angles,
                Kind::RailAbs,
                Kind::Ik,
                Kind::Pose,
                Kind::Swing,
            ] {
                ui.selectable_value(&mut app.draft.kind, kind, kind.label());
            }
        });

    let (rail_min, rail_max) = rail_range(&app.arm);

    match app.draft.kind {
        Kind::Joint => {
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
        Kind::Angles => {
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
        Kind::RailAbs => {
            ranged(
                ui,
                "레일 위치 [m]",
                &mut app.draft.rail_x,
                rail_min,
                rail_max,
                0.005,
            );
        }
        Kind::Ik => {
            draw_reach(ui, app, false);
        }
        Kind::Pose => {
            draw_reach(ui, app, true);
        }
        Kind::Swing => {
            draw_swing(ui, app, preview);
        }
    }
}

fn draw_swing(ui: &mut egui::Ui, app: &mut JogApp, preview: Option<&SwingPreview>) {
    let hit = InterceptWindow::default();
    ui.label("공을 맞을 깊이 (접수 평면 y) [m]");
    ranged(
        ui,
        "y",
        &mut app.draft.hit_plane_y,
        hit.y_min,
        hit.y_max,
        0.005,
    );

    ui.label("면 기울기 [°]");
    ranged(ui, "pitch", &mut app.draft.tilt_pitch_deg, -30.0, 30.0, 0.5);
    ranged(ui, "yaw", &mut app.draft.tilt_yaw_deg, -30.0, 30.0, 0.5);

    ui.separator();
    if app.synced_pose.is_none() {
        ui.label("동기화하면 예측 결과가 표시됩니다");
        return;
    }
    let Some(preview) = preview else {
        ui.colored_label(
            Color32::from_rgb(220, 90, 80),
            "이 슈터 설정으로는 접수 평면에 도달하는 공이 없습니다",
        );
        ui.label(
            RichText::new("네트 미달 · 너무 낮음 · 리드 시간 밖 — 속도나 pitch를 올려보세요")
                .weak()
                .small(),
        );
        return;
    };

    let p = preview.prediction.impact_position.coords;
    let v = preview.prediction.incoming_velocity;
    ui.label(format!("도달점 = ({:.3}, {:.3}, {:.3}) m", p.x, p.y, p.z));
    ui.label(format!("입사 속도 = ({:.2}, {:.2}, {:.2}) m/s", v.x, v.y, v.z));
    ui.label(format!(
        "리드 시간 = {:.3} s",
        preview.prediction.time_to_impact_secs
    ));
    if preview.ik_ok {
        ui.colored_label(Color32::from_rgb(90, 190, 120), "IK 가능");
    } else {
        ui.colored_label(
            Color32::from_rgb(220, 90, 80),
            "IK 불가 — 깊이·기울기나 슈터 조준을 바꿔보세요",
        );
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

fn draw_actions(ui: &mut egui::Ui, app: &mut JogApp, preview: Option<&SwingPreview>) {
    // 슈터 공이 도달 불가이거나 임팩트 IK가 안 풀리면 미리보기를 막는다.
    let swing_ready = app.draft.kind != Kind::Swing || preview.is_some_and(|p| p.ik_ok);
    ui.horizontal(|ui| {
        if ui
            .add_enabled(
                app.phase.can_preview() && swing_ready,
                egui::Button::new("미리보기"),
            )
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

fn joint_hw_deg_range(arm: &pingpong_bot::robot::Arm, index: usize) -> (f64, f64) {
    if let Some(limit) = arm.joint_limit(index) {
        return (limit.min.to_degrees(), limit.max.to_degrees());
    }
    return (-180.0, 180.0);
}

/// 관절 한계 ∩ (현재각 ± maxdelta). 슬라이더가 maxdelta 위반을 미리 막는다.
fn joint_jog_deg_range(
    arm: &pingpong_bot::robot::Arm,
    index: usize,
    synced: Option<&pingpong_bot::robot::Pose>,
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

fn rail_range(arm: &pingpong_bot::robot::Arm) -> (f64, f64) {
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
