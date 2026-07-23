//! egui 슈터·sim 제어 패널 (kiss3d `draw_ui` 오버레이).
//!
//! 역할별 작은 창으로 나눠 3D 시야를 덜 가린다.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use kiss3d::egui;

use super::debug_overlays::DebugOverlays;
use super::debug_snap::CommitPhase;
use crate::Prediction;
use crate::constants::viewer::{CAMERA_DIST_DEFAULT, CAMERA_DIST_MAX, CAMERA_DIST_MIN};
use crate::defaults;
use crate::sim::physics::shooter::{BallShooterSettings, BallState};
use crate::sim::physics::world::SimWorld;
use crate::sim::session::controls::SimRuntimeControls;

/// 한글 글리프용 폰트 (NanumGothic, OFL). 한 번만 설치.
static KOREAN_FONTS_INSTALLED: AtomicBool = AtomicBool::new(false);

/// egui 기본 폰트에 한글 폴백을 넣는다. `draw_ui`마다 호출해도 안전.
pub fn ensure_korean_fonts(ctx: &egui::Context) {
    if KOREAN_FONTS_INSTALLED.swap(true, Ordering::Relaxed) {
        return;
    }
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "NanumGothic".to_owned(),
        egui::FontData::from_static(include_bytes!("../../../assets/fonts/NanumGothic-Regular.ttf"))
            .into(),
    );
    // Latin은 기본 폰트, 한글 글리프만 NanumGothic으로 폴백.
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

/// 패널 슬라이더 상태 — 매 프레임 `controls` 락 없이 UI를 그린다.
#[derive(Clone, Debug)]
pub struct PanelUiState {
    pub shooter: BallShooterSettings,
    pub time_scale: f64,
    /// OrbitCamera3d 거리 [m]
    pub camera_dist: f32,
    pub debug: DebugOverlays,
}

impl PanelUiState {
    pub fn from_controls(controls: &SimRuntimeControls) -> Self {
        return Self {
            shooter: controls.shooter.clone(),
            time_scale: controls.time_scale,
            camera_dist: CAMERA_DIST_DEFAULT,
            debug: DebugOverlays::debug_defaults(),
        };
    }
}

/// 상태 표시용 스냅샷 — world 락을 메인 스레드에서 잡지 않기 위함.
#[derive(Clone, Debug)]
pub struct StatusSnapshot {
    pub ball_state: BallState,
    pub sim_time: f64,
    pub ball_pos: (f32, f32, f32),
    pub ball_vel: (f32, f32, f32),
    pub joints: Vec<String>,
    /// hit plane 예측 (디버그)
    pub debug_prediction: Option<Prediction>,
    pub swing_committed: bool,
    pub swing_abandoned: bool,
    pub last_fail_text: Option<String>,
    pub unreachable_xyz: Option<[f64; 3]>,
    pub commit_phase: CommitPhase,
    pub table_pen_depth: f64,
    pub torque_over: Vec<bool>,
    pub accel_over: bool,
    pub joint_at_limit: Vec<bool>,
    pub omega: [f64; 3],
    pub net_gate_ok: Option<bool>,
}

impl StatusSnapshot {
    /// 월드에서 한 프레임 분량의 상태를 읽는다.
    pub fn from_world(world: &SimWorld) -> Self {
        let bp = world.ball_position();
        let bv = world.ball_velocity();
        let snap = world.debug_snap();
        return Self {
            ball_state: world.ball_state,
            sim_time: world.sim_time,
            ball_pos: (bp.x, bp.y, bp.z),
            ball_vel: (bv.x, bv.y, bv.z),
            joints: world
                .robot()
                .joints()
                .values
                .iter()
                .map(|v| format!("{v:.2}"))
                .collect(),
            debug_prediction: world.debug_prediction().cloned(),
            swing_committed: world.swing_committed(),
            swing_abandoned: world.swing_abandoned(),
            last_fail_text: snap.last_fail_text.clone(),
            unreachable_xyz: snap.unreachable_xyz,
            commit_phase: snap.commit_phase,
            table_pen_depth: snap.table_pen_depth,
            torque_over: snap.torque_over.clone(),
            accel_over: snap.accel_over,
            joint_at_limit: snap.joint_at_limit.clone(),
            omega: snap.omega,
            net_gate_ok: snap.net_gate_ok,
        };
    }
}

/// 슈터 설정·상태 패널 (역할별 창).
pub fn draw(
    ctx: &egui::Context,
    ui_state: &mut PanelUiState,
    controls: &Arc<Mutex<SimRuntimeControls>>,
    status: Option<&StatusSnapshot>,
) {
    ensure_korean_fonts(ctx);

    let mut shoot = false;
    let mut random_shoot = false;
    let mut park = false;

    egui::Window::new("Shooter")
        .default_width(260.0)
        .default_pos([12.0, 12.0])
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            ui.collapsing("Position", |ui| {
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.pos_offset_x_m, -0.8..=0.8)
                        .text("x [m]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.pos_offset_y_m, -0.6..=0.8)
                        .text("y [m]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.pos_offset_z_m, -0.3..=0.5)
                        .text("z [m]"),
                );
                let m = ui_state.shooter.mount_position();
                ui.monospace(format!("mount {:.2} {:.2} {:.2}", m.x, m.y, m.z));
            });
            ui.collapsing("Aim", |ui| {
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.yaw_deg, -25.0..=25.0)
                        .text("yaw [deg]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.pitch_deg, -25.0..=25.0)
                        .text("pitch [deg]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.roll_deg, -45.0..=45.0)
                        .text("roll [deg]"),
                );
            });
            ui.collapsing("Muzzle", |ui| {
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.lateral_offset_m, -0.5..=0.5)
                        .text("lateral [m]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.height_offset_m, -0.2..=0.4)
                        .text("height [m]"),
                );
            });
            ui.collapsing("Speed / spin", |ui| {
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.speed_mps, 3.0..=15.0)
                        .text("speed [m/s]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.topspin_rad_s, -80.0..=80.0)
                        .text("topspin"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.sidespin_rad_s, -80.0..=80.0)
                        .text("sidespin"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.drill_spin_rad_s, -80.0..=80.0)
                        .text("drill"),
                );
            });
            ui.horizontal(|ui| {
                if ui.button("Shoot").clicked() {
                    shoot = true;
                }
                if ui.button("Random").clicked() {
                    random_shoot = true;
                }
                if ui.button("Park").clicked() {
                    park = true;
                }
            });
        });

    // Status를 위에, View(체크박스 많음)를 아래에 — View가 Status를 가리지 않게.
    egui::Window::new("Status")
        .default_width(280.0)
        .anchor(egui::Align2::RIGHT_TOP, [-12.0, 12.0])
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            let Some(status) = status else {
                ui.label("월드 연결 대기…");
                return;
            };
            draw_status_panel(ui, status, &ui_state.debug);
        });

    egui::Window::new("View")
        .default_width(220.0)
        .anchor(egui::Align2::RIGHT_TOP, [-12.0, 340.0])
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            ui.add(
                egui::Slider::new(&mut ui_state.time_scale, 0.1..=20.0)
                    .logarithmic(true)
                    .text("배속 (1=실시간)"),
            );
            ui.add(
                egui::Slider::new(&mut ui_state.camera_dist, CAMERA_DIST_MIN..=CAMERA_DIST_MAX)
                    .text("zoom [m]"),
            );
            ui.small("drag=orbit · scroll=zoom");
            ui.small("axes: R=X  G=Y  B=Z");
            ui.separator();
            ui.label("Debug overlays");
            ui.small("항목에 마우스를 올리면 설명");
            ui.horizontal(|ui| {
                if ui.small_button("defaults").clicked() {
                    ui_state.debug = DebugOverlays::debug_defaults();
                }
                if ui.small_button("all off").clicked() {
                    ui_state.debug = DebugOverlays::all_off();
                }
            });
            let d = &mut ui_state.debug;
            debug_checkbox(ui, &mut d.impact_markers, "impact markers", |ui| {
                ui.strong("예상 타격점");
                ui.label("공이 라켓에 맞을 것으로 예측한 위치입니다.");
                ui.add_space(4.0);
                ui.label("· 반투명 벽 — 접수 평면");
                ui.label("· 작은 구체 — 예상 타격점");
                ui.label("· 노란 판 — 테이블 위 투영");
                ui.add_space(4.0);
                ui.strong("색 의미");
                ui.label("분홍 — 아직 스윙 결정 전 (예측만)");
                ui.label("초록 — 스윙 계획 확정");
                ui.label("빨강 — 팔이 그 지점에 닿지 않음");
                ui.label("주황 — 치기까지 시간이 너무 짧음");
                ui.label("보라 — 원하는 리턴 속도를 만들 수 없음");
                ui.label("청록 — 자세가 테이블을 뚫음");
                ui.label("노랑 — 관절·토크·레일 한계 초과");
            });
            debug_checkbox(ui, &mut d.fail_status, "fail status", |ui| {
                ui.strong("실패 사유 (Status)");
                ui.label("마지막에 스윙을 포기하거나 건너뛴 이유를");
                ui.label("Status 창에 한글로 보여 줍니다.");
                ui.label("목표 좌표가 있으면 같이 표시됩니다.");
            });
            debug_checkbox(ui, &mut d.unreachable_x, "unreachable X", |ui| {
                ui.strong("도달 불가 목표");
                ui.label("팔이 닿지 않거나 한계에 걸린 목표점에");
                ui.label("빨간 X를 그립니다.");
            });
            debug_checkbox(ui, &mut d.joint_limits, "joint limits", |ui| {
                ui.strong("관절 한계");
                ui.label("관절이 가동 범위 끝에 닿으면");
                ui.label("해당 링크·조인트를 빨갛게 표시합니다.");
            });
            debug_checkbox(ui, &mut d.torque_hud, "torque HUD", |ui| {
                ui.strong("토크·가속 경고 (Status)");
                ui.label("Status에 다음을 표시합니다.");
                ui.label("· 토크/가속 한계 초과");
                ui.label("· 관절이 리밋에 닿음");
                ui.label("· 테이블 침투 깊이");
            });
            debug_checkbox(ui, &mut d.commit_bar, "commit bar", |ui| {
                ui.strong("스윙 결정 타이밍");
                ui.label("임팩트까지 남은 시간(tti)이");
                ui.label("스윙을 확정해도 되는 구간인지");
                ui.label("Status에 막대로 보여 줍니다.");
                ui.label("(너무 이르면 대기, 너무 늦으면 포기)");
            });
            debug_checkbox(ui, &mut d.table_obb, "table OBB", |ui| {
                ui.strong("테이블 침투 박스");
                ui.label("팔이나 라켓이 테이블 안으로 들어가면");
                ui.label("그 부분을 반투명 빨간 박스로 강조합니다.");
            });
            debug_checkbox(ui, &mut d.net_gate, "net gate tone", |ui| {
                ui.strong("네트 미달");
                ui.label("예측 탄도가 네트보다 낮게 지나가면");
                ui.label("공을 회색으로 바꾸고 Status에 표시합니다.");
            });
            debug_checkbox(ui, &mut d.predicted_arc, "predicted arc", |ui| {
                ui.strong("예측 탄도");
                ui.label("물리 모델이 그린 공의 예상 경로입니다.");
                ui.label("(하늘색 점)");
            });
            debug_checkbox(ui, &mut d.truth_arc, "truth arc", |ui| {
                ui.strong("실제 탄도");
                ui.label("시뮬레이터가 실제로 움직인 공의 경로입니다.");
                ui.label("(주황 점) 예측과 비교해 보세요.");
            });
            debug_checkbox(ui, &mut d.swing_ghost, "swing ghost", |ui| {
                ui.strong("스윙 경로");
                ui.label("확정된 스윙에서 라켓 중심이 지나갈");
                ui.label("경로를 회색 점으로 그립니다.");
            });
            debug_checkbox(ui, &mut d.rail_stroke, "rail stroke", |ui| {
                ui.strong("레일 이동 범위");
                ui.label("레일이 갈 수 있는 양쪽 끝과");
                ui.label("지금 위치를 표시합니다.");
            });
            debug_checkbox(ui, &mut d.aim_band, "aim band", |ui| {
                ui.strong("Random 조준 대역");
                ui.label("Random Shoot이 겨냥하는");
                ui.label("로봇 쪽 테이블 가장자리(y≈0) 구간입니다.");
                ui.label("양끝 padding 안쪽만 조준합니다.");
            });
            debug_checkbox(ui, &mut d.omega_arrow, "ω arrow", |ui| {
                ui.strong("스핀 방향");
                ui.label("공의 각속도(스핀) 방향을 화살표로 그립니다.");
                ui.label("탑스핀·사이드스핀이 어느 쪽인지 볼 수 있습니다.");
            });
        });

    if let Ok(mut ctrl) = controls.try_lock() {
        // Random은 슬라이더(`ui_state.shooter`)에도 반영한다 — 안 그러면 다음
        // 프레임에 원본으로 덮여 슈터 위치가 한 프레임만 깜빡인다.
        if random_shoot {
            ui_state.shooter = ui_state.shooter.randomized(&mut rand::thread_rng());
            ctrl.request_shoot();
        }
        ctrl.shooter = ui_state.shooter.clone();
        ctrl.time_scale = ui_state.time_scale;
        if shoot {
            ctrl.request_shoot();
        }
        if park {
            ctrl.request_park();
        }
    }
}

fn debug_checkbox(
    ui: &mut egui::Ui,
    value: &mut bool,
    label: &str,
    tip: impl FnOnce(&mut egui::Ui),
) {
    ui.checkbox(value, label).on_hover_ui(tip);
}

fn draw_status_panel(ui: &mut egui::Ui, status: &StatusSnapshot, debug: &DebugOverlays) {
    let ball_ko = match status.ball_state {
        BallState::Parked => "주차 (슈터에 대기)",
        BallState::InFlight => "비행 중",
    };
    let swing_ko = if status.swing_committed {
        "확정 — 치는 중"
    } else if status.swing_abandoned {
        "포기 — 이번 공은 안 침"
    } else {
        "대기"
    };

    ui.strong("공");
    ui.label(format!("상태  {ball_ko}"));
    ui.label(format!("시뮬 시간  {:.2} s", status.sim_time));
    ui.label(format!(
        "위치 [m]  x {:.2}  y {:.2}  z {:.2}",
        status.ball_pos.0, status.ball_pos.1, status.ball_pos.2
    ));
    ui.label(format!(
        "속도 [m/s]  x {:.2}  y {:.2}  z {:.2}",
        status.ball_vel.0, status.ball_vel.1, status.ball_vel.2
    ));
    if debug.omega_arrow || debug.fail_status {
        let w = status.omega;
        let mag = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        ui.label(format!(
            "스핀 [rad/s]  크기 {mag:.1}  ({:.0}, {:.0}, {:.0})",
            w[0], w[1], w[2]
        ));
    }

    ui.separator();
    ui.strong("로봇");
    ui.label(format!("관절각 [rad]  {}", status.joints.join("  ")));
    ui.label(format!("스윙  {swing_ko}"));
    ui.label(format!("단계  {}", status.commit_phase.label_ko()));

    ui.separator();
    ui.strong("예상 타격");
    if let Some(pred) = &status.debug_prediction {
        let p = pred.impact_position.coords;
        ui.label(format!("임팩트까지  {:.3} s", pred.time_to_impact_secs));
        ui.label(format!(
            "예상 위치 [m]  x {:.2}  y {:.2}  z {:.2}",
            p.x, p.y, p.z
        ));
    } else {
        ui.small("아직 예측 없음 (공이 날아오면 표시)");
    }

    if debug.commit_bar {
        draw_commit_bar(ui, status);
    }
    if debug.net_gate {
        match status.net_gate_ok {
            Some(true) => {
                ui.label("네트  통과 가능");
            }
            Some(false) => {
                ui.colored_label(egui::Color32::GRAY, "네트  높이 미달 — 접수 불가");
            }
            None => {}
        }
    }

    if debug.fail_status {
        if let Some(text) = &status.last_fail_text {
            ui.separator();
            ui.strong("최근 실패");
            ui.colored_label(egui::Color32::from_rgb(255, 120, 90), text);
        }
        if let Some([x, y, z]) = status.unreachable_xyz {
            ui.label(format!("목표점 [m]  x {x:.2}  y {y:.2}  z {z:.2}"));
        }
    }

    if debug.torque_hud {
        let mut any = false;
        if status.accel_over {
            if !any {
                ui.separator();
                ui.strong("한계 경고");
                any = true;
            }
            ui.colored_label(egui::Color32::YELLOW, "관절 가속이 허용 상한을 넘김");
        }
        if status.torque_over.iter().any(|&o| o) {
            if !any {
                ui.separator();
                ui.strong("한계 경고");
                any = true;
            }
            let axes: Vec<String> = status
                .torque_over
                .iter()
                .enumerate()
                .filter(|(_, o)| **o)
                .map(|(i, _)| format!("관절 {i}"))
                .collect();
            ui.colored_label(
                egui::Color32::YELLOW,
                format!("토크 초과 — {}", axes.join(", ")),
            );
        }
        if status.joint_at_limit.iter().any(|&o| o) {
            if !any {
                ui.separator();
                ui.strong("한계 경고");
                any = true;
            }
            let axes: Vec<String> = status
                .joint_at_limit
                .iter()
                .enumerate()
                .filter(|(_, o)| **o)
                .map(|(i, _)| format!("관절 {i}"))
                .collect();
            ui.colored_label(
                egui::Color32::from_rgb(255, 80, 80),
                format!("관절 가동범위 끝 — {}", axes.join(", ")),
            );
        }
        if status.table_pen_depth > 1e-4 {
            if !any {
                ui.separator();
                ui.strong("한계 경고");
            }
            ui.colored_label(
                egui::Color32::from_rgb(80, 220, 230),
                format!("테이블 침투  {:.1} mm", status.table_pen_depth * 1000.0),
            );
        }
    }
}

fn draw_commit_bar(ui: &mut egui::Ui, status: &StatusSnapshot) {
    let control = defaults::control();
    let min_s = control.min_swing_secs;
    let max_s = control.swing_commit_max_secs;
    let tti = status
        .debug_prediction
        .as_ref()
        .map(|p| p.time_to_impact_secs);
    let Some(tti) = tti else {
        ui.label(format!(
            "스윙 확정 구간  {min_s:.2}–{max_s:.2} s (임팩트까지 남은 시간)"
        ));
        return;
    };
    let span = (max_s - min_s).max(1e-6);
    let frac = ((tti - min_s) / span).clamp(0.0, 1.0);
    let filled = (frac * 10.0).round() as usize;
    let mut bar = String::from("[");
    for i in 0..10 {
        bar.push(if i < filled { '=' } else { ' ' });
    }
    bar.push(']');
    let mark = if (min_s..=max_s).contains(&tti) {
        "지금 확정해도 됨"
    } else if tti > max_s {
        "아직 이름 — 대기"
    } else {
        "너무 늦음"
    };
    ui.label(format!("남은 시간  {tti:.3} s  {bar}"));
    ui.label(format!("확정 창  {min_s:.2}–{max_s:.2} s  ·  {mark}"));
}
