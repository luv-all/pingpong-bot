//! egui 패널 — Shooter / Eval / Status / View 역할별 창.

pub use super::panel_ui_state::PanelUiState;
pub use super::status_snapshot::StatusSnapshot;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use kiss3d::egui;

use super::super::debug::overlays::DebugOverlays;
use super::eval_live_run::EvalLiveRun;
use crate::constants::viewer::{CAMERA_DIST_MAX, CAMERA_DIST_MIN};
use crate::defaults;
use crate::robot::motion;
use crate::robot::{self, Joints, Robot};
use crate::sim::eval;
use crate::sim::physics;
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
        egui::FontData::from_static(include_bytes!(
            "../../../../assets/fonts/NanumGothic-Regular.ttf"
        ))
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

pub fn draw(
    ctx: &egui::Context,
    ui_state: &mut PanelUiState,
    controls: &Arc<Mutex<SimRuntimeControls>>,
    world: &Arc<Mutex<SimWorld>>,
    status: Option<&StatusSnapshot>,
    // 관절별 스크린 좌표 (egui, top-left). `None`이면 해당 관절 숨김.
    joint_screen: Option<&[Option<egui::Pos2>]>,
) {
    ensure_korean_fonts(ctx);

    if ui_state.debug.joint_anchors
        && let (Some(status), Some(screens)) = (status, joint_screen)
    {
        draw_joint_anchor_windows(ctx, status, screens);
    }

    let mut shoot = false;
    let mut random_shoot = false;
    let mut park = false;
    let mut start_eval = false;
    let mut start_eval_mode = eval::Mode::Block;
    let mut start_live_shot: Option<usize> = None;
    let mut joint_test_requested = false;
    let mut joint_test_copy_start = false;
    let mut joint_test_copy_end = false;

    // 레이아웃: 좌측 Shooter→Rig→Eval, 우측 Status→View.
    //
    // Rig가 좌측에 있는 이유: Shooter와 Rig는 둘 다 "리그를 어디에 놓았나"다
    // (슈터 위치 / 로봇 마운트 위치). 우측은 읽기 전용 상태·보기 설정이라
    // 조정 손잡이는 좌측에 모은다.
    const GUI_GAP: f32 = 12.0;
    let screen = ctx.content_rect();

    let shooter_win = egui::Window::new("Shooter")
        .default_width(260.0)
        .default_pos(screen.left_top() + egui::vec2(12.0, 12.0))
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            let buttons = crate::sim::gui::shooter::ui::draw(
                ui,
                &mut ui_state.shooter,
                crate::sim::gui::shooter::ui::ButtonSet::ALL,
            );
            shoot |= buttons.shoot;
            random_shoot |= buttons.random;
            park |= buttons.park;
        });

    let rig_y = shooter_win
        .as_ref()
        .map(|r| r.response.rect.bottom() + GUI_GAP)
        .unwrap_or(screen.top() + 320.0);
    let rig_win = egui::Window::new("Rig")
        .default_width(260.0)
        .default_pos(egui::pos2(screen.left() + 12.0, rig_y))
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            draw_rig_panel(ui, ui_state, status.map(|s| s.ball_state));
        });

    let eval_y = rig_win
        .as_ref()
        .map(|r| r.response.rect.bottom() + GUI_GAP)
        .unwrap_or(rig_y + 160.0);
    let eval_win = egui::Window::new("Eval")
        .default_width(260.0)
        .default_pos(egui::pos2(screen.left() + 12.0, eval_y))
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            let running = ui_state.eval_running.load(Ordering::Relaxed);
            ui.add_enabled_ui(!running, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Mode");
                    ui.selectable_value(&mut ui_state.eval_mode, eval::Mode::Block, "Block");
                    ui.selectable_value(
                        &mut ui_state.eval_mode,
                        eval::Mode::Alternating,
                        "Alternating",
                    );
                });
                ui.weak(match ui_state.eval_mode {
                    eval::Mode::Block => "Left → Center → Right · 10 each",
                    eval::Mode::Alternating => "L → C → R → C · … · 10 each",
                });
                ui.add_space(4.0);
                ui.add(
                    egui::Slider::new(&mut ui_state.eval_launch.speed_mps, 3.0..=15.0)
                        .text("speed [m/s]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.eval_launch.side_yaw_deg, 0.0..=25.0)
                        .text("side yaw [deg]"),
                );
                ui.weak("L/R = ±yaw · Center = 0 (Shooter와 별도)");
                ui.add_space(4.0);
                let run =
                    egui::Button::new("Run 30").min_size(egui::vec2(ui.available_width(), 22.0));
                if ui.add(run).clicked() {
                    start_eval = true;
                    start_eval_mode = ui_state.eval_mode;
                }
            });
            draw_eval_status(ui, ui_state, &mut start_live_shot);
        });

    let motor_test_y = eval_win
        .as_ref()
        .map(|r| r.response.rect.bottom() + GUI_GAP)
        .unwrap_or(eval_y + 160.0);
    // 관절 한계[deg] — 입력을 이 범위로 직접 잘라서, 범위 밖 값을 넣고 조용히
    // "InfeasibleSwing"으로 실패하는(로봇은 안 움직이는데 이유를 모르는) 경우를
    // 애초에 막는다. yaw(관절0)는 continuous라 한계가 없다.
    let joint_test_limits_deg: [Option<(f64, f64)>; 4] = {
        let mut limits = [None; 4];
        if let Ok(w) = world.try_lock() {
            for (i, limit) in limits.iter_mut().enumerate() {
                *limit = w
                    .arm
                    .joint_limit(i)
                    .map(|l| (l.min.to_degrees(), l.max.to_degrees()));
            }
        }
        limits
    };
    egui::Window::new("Motor Test")
        .default_width(280.0)
        .default_pos(egui::pos2(screen.left() + 12.0, motor_test_y))
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            draw_motor_test_panel(
                ui,
                ui_state,
                &joint_test_limits_deg,
                &mut joint_test_requested,
                &mut joint_test_copy_start,
                &mut joint_test_copy_end,
            );
        });

    let status_win = egui::Window::new("Status")
        .default_width(280.0)
        .pivot(egui::Align2::RIGHT_TOP)
        .default_pos(screen.right_top() + egui::vec2(-12.0, 12.0))
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            let Some(status) = status else {
                ui.label("월드 연결 대기…");
                return;
            };
            draw_status_panel(ui, status, &ui_state.debug);
        });

    let view_y = status_win
        .as_ref()
        .map(|r| r.response.rect.bottom() + GUI_GAP)
        .unwrap_or(screen.top() + 420.0);
    egui::Window::new("View")
        .default_width(240.0)
        .pivot(egui::Align2::RIGHT_TOP)
        .default_pos(egui::pos2(screen.right() - 12.0, view_y))
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            draw_view_panel(ui, ui_state);
        });

    if let Ok(mut ctrl) = controls.try_lock() {
        // Random은 슬라이더(`ui_state.shooter`)에도 반영한다 — 안 그러면 다음
        // 프레임에 원본으로 덮여 슈터 위치가 한 프레임만 깜빡인다.
        if random_shoot {
            ui_state.shooter = ui_state
                .shooter
                .randomized_at_current_muzzle(&mut rand::thread_rng());
            ctrl.request_shoot();
        }
        ctrl.shooter = ui_state.shooter.clone();
        ctrl.rail_frame = ui_state.rail_frame;
        ctrl.intercept = ui_state.intercept;
        ctrl.time_scale = ui_state.time_scale;
        ctrl.use_bang_bang_swing = ui_state.use_bang_bang_swing;
        ctrl.use_fixed_swing_dictionary = ui_state.use_fixed_swing_dictionary;
        ctrl.fixed_swing_impact_strategy = ui_state.fixed_swing_impact_strategy;
        if shoot {
            ctrl.request_shoot();
            // Motor Test가 켰을 수 있는 키네마틱 미리보기·꺼둔 자동 복귀를 정식
            // 랠리 전에 되살린다 — 안 그러면 물리 기반 PD 추종(진짜 랠리 경로)이
            // 아니라 키네마틱 텔레포트로 실제 공을 받으려 들거나, 스윙 뒤 팔이
            // 중앙으로 안 돌아와 다음 공을 못 받는다.
            if let Ok(mut w) = world.lock() {
                w.set_kinematic_robot(false);
                w.robot_mut().set_auto_return_to_center(true);
            }
        }
        if park {
            ctrl.request_park();
        }
    }

    if start_eval {
        ui_state.eval_live = None;
        start_eval_protocol(ui_state, world, start_eval_mode);
    }
    if let Some(shot_number) = start_live_shot {
        begin_eval_live_shot(ui_state, world, controls, shot_number);
    }

    if (joint_test_copy_start || joint_test_copy_end)
        && let Some(status) = status
    {
        for (i, q) in status.joint_q.iter().enumerate().take(4) {
            if joint_test_copy_start {
                ui_state.joint_test_start_deg[i] = q.to_degrees();
            }
            if joint_test_copy_end {
                ui_state.joint_test_end_deg[i] = q.to_degrees();
            }
        }
    }
    if joint_test_requested {
        run_joint_motor_test(ui_state, world);
    }
}

/// "Motor Test" 창 내용 — IK 없이 4관절 시작/끝 각[deg]을 직접 입력해
/// [`motion::Planner::move_to_fastest`](정지→정지, 부드러운 재추종용 시간
/// 하한 없이 모터 한계로만 정한 최단 실행가능 quintic)로 재생한다.
fn draw_motor_test_panel(
    ui: &mut egui::Ui,
    ui_state: &mut PanelUiState,
    joint_limits_deg: &[Option<(f64, f64)>; 4],
    joint_test_requested: &mut bool,
    joint_test_copy_start: &mut bool,
    joint_test_copy_end: &mut bool,
) {
    ui.small("IK 없이 4관절 각[deg]을 직접 지정 — start로 스냅 후 end까지 모터 한계(속도·가속·토크) 100%로 최단 시간 이동");
    ui.small("end 도달 후 자동 중앙 복귀를 끔 — 결과 자세 확인용. Shoot을 누르면 다시 켜짐");
    egui::Grid::new("motor_test_grid")
        .num_columns(5)
        .spacing([6.0, 4.0])
        .show(ui, |ui| {
            ui.label("");
            for label in ["j0 yaw", "j1 shoulder", "j2 elbow", "j3 wrist"] {
                ui.weak(label);
            }
            ui.end_row();

            ui.label("start");
            for (v, limit) in ui_state
                .joint_test_start_deg
                .iter_mut()
                .zip(joint_limits_deg)
            {
                add_bounded_drag_value(ui, v, *limit);
            }
            ui.end_row();

            ui.label("end");
            for (v, limit) in ui_state.joint_test_end_deg.iter_mut().zip(joint_limits_deg) {
                add_bounded_drag_value(ui, v, *limit);
            }
            ui.end_row();

            ui.label("");
            for limit in joint_limits_deg {
                match limit {
                    Some((lo, hi)) => ui.weak(format!("{lo:.0}°…{hi:.0}°")),
                    None => ui.weak("한계 없음"),
                };
            }
            ui.end_row();
        });
    ui.horizontal(|ui| {
        if ui.small_button("현재 포즈 → start").clicked() {
            *joint_test_copy_start = true;
        }
        if ui.small_button("현재 포즈 → end").clicked() {
            *joint_test_copy_end = true;
        }
    });
    ui.add_space(4.0);
    let test_button =
        egui::Button::new("Test — 모터 100% 속도").min_size(egui::vec2(ui.available_width(), 22.0));
    if ui.add(test_button).clicked() {
        *joint_test_requested = true;
    }
    if let Some(duration) = ui_state.joint_test_last_duration_secs {
        ui.weak(format!("계획된 소요 시간: {duration:.3} s"));
    }
    if let Some(err) = &ui_state.joint_test_error {
        ui.colored_label(egui::Color32::from_rgb(220, 90, 80), err);
    }
}

/// start 포즈로 즉시 스냅한 뒤, end까지 [`motion::Planner::move_to_fastest`]로
/// 계획한 궤적을 커밋한다 — 레일은 현재 위치에 고정(4관절 모터만 대상).
fn run_joint_motor_test(ui_state: &mut PanelUiState, world: &Arc<Mutex<SimWorld>>) {
    let Ok(mut w) = world.lock() else {
        return;
    };
    let rail_x = w.robot.rail_x();
    let arm = Arc::clone(&w.arm);
    let start_rad: Vec<f64> = ui_state
        .joint_test_start_deg
        .iter()
        .map(|d| d.to_radians())
        .collect();
    let end_rad: Vec<f64> = ui_state
        .joint_test_end_deg
        .iter()
        .map(|d| d.to_radians())
        .collect();
    let start_pose = robot::Pose::new(rail_x, Joints::from_slice(&start_rad));
    // 메인 sim은 기본적으로 물리(Rapier PD 모터) 경로다 — 토크·속도 한계를
    // 그대로 받는 진짜 랠리 재생에는 맞지만, 모터 순수 성능만 보고 싶은 이
    // 테스트에는 PD 추종 지연·정합성 문제가 낀다. `tools/jog`가 수동 미리보기에
    // 쓰는 것과 같은 키네마틱 모드로 돌린다 — 매 틱 `state.angles`를 궤적
    // 샘플로 직접 덮어쓰고 다물체를 그 값으로 텔레포트하므로(`step_kinematic_robot`),
    // 화면에 궤적 모양 그대로 보인다. Shoot을 누르면 물리 모드로 되돌아간다.
    w.set_kinematic_robot(true);
    // `state.snap_to_pose`(State만)가 아니라 `SimWorld::snap_robot_pose`를 써야
    // 한다 — 후자만 Rapier 다물체(실제 팔 위치)까지 함께 텔레포트한다. State만
    // 스냅하면 다음 물리 스텝의 `set_measured_joints`가 실제 다물체 위치(옛
    // 자세)로 곧바로 덮어써서, "start"가 화면엔 한 프레임만 반짝이고 실제로는
    // 옛 위치에서 end로 쫓아가 버린다 — end가 옛 위치와 가까우면 거의 안
    // 움직이는 것처럼 보인다.
    w.snap_robot_pose(start_pose.clone());
    // 기본 랠리 동작(스윙 끝나면 중앙 복귀)이 end 도달 직후 곧바로 되돌려버리면
    // 결과 자세를 확인할 새가 없다 — Shoot을 누르기 전까지 꺼 둔다.
    w.robot_mut().set_auto_return_to_center(false);

    match motion::Planner::move_to_fastest(&arm, &start_pose, Joints::from_slice(&end_rad), rail_x)
    {
        Ok(trajectory) => {
            ui_state.joint_test_last_duration_secs = Some(trajectory.duration_secs);
            ui_state.joint_test_error = None;
            w.robot_mut().replace_swing(trajectory);
        }
        Err(err) => {
            ui_state.joint_test_last_duration_secs = None;
            ui_state.joint_test_error = Some(format!("{err}"));
        }
    }
}

/// 레일 리그 설치 위치 — 공이 주차된 동안만 조정 가능.
///
/// 두께([`RAIL_THICKNESS`](crate::constants::geometry::RAIL_THICKNESS))는 실측
/// 고정이라 슬라이더로 내놓지 않는다. 실물에서 못 바꾸는 값을 시뮬에서만
/// 만질 수 있게 하면 시뮬이 도달 못 하는 자세를 낼 수 있다고 착각하게 된다.
///
/// Y는 현장에서 줄자로 재는 "탁구대 끝선-레일 거리"를 양수로 보여주고,
/// Z는 월드 바닥 기준 레일 하단 높이를 보여준다. 내부 기구학 좌표에서는
/// 테이블 밖이 -Y이므로
/// [`RailFrame::set_table_distance_m`](crate::robot::RailFrame::set_table_distance_m)가
/// 부호를 한 곳에서 변환한다.
fn draw_rig_panel(
    ui: &mut egui::Ui,
    ui_state: &mut PanelUiState,
    ball_state: Option<physics::BallState>,
) {
    let parked = ball_state == Some(physics::BallState::Parked);
    let PanelUiState {
        rail_frame: frame,
        intercept,
        ..
    } = ui_state;

    ui.add_enabled_ui(parked, |ui| {
        let mut table_distance = frame.table_distance_m();
        if ui
            .add(egui::Slider::new(&mut table_distance, 0.00..=0.50).text("탁구대-레일 거리 [m]"))
            .changed()
        {
            frame.set_table_distance_m(table_distance);
        }
        ui.add(egui::Slider::new(&mut frame.rail_bottom_z, 0.70..=1.10).text("레일 하단 z [m]"));
        ui.separator();
        ui.strong("타격 후보 범위");
        ui.add(egui::Slider::new(&mut intercept.y_min, 0.00..=0.80).text("시작 Y [m]"));
        ui.add(egui::Slider::new(&mut intercept.y_max, intercept.y_min..=1.00).text("끝 Y [m]"));
        ui.add(egui::Slider::new(&mut intercept.sample_step, 0.01..=0.10).text("간격 [m]"));
    });

    ui.add_space(4.0);
    ui.monospace(format!(
        "거리 {:.3} m  면 위 {:+.3} m  후보 {}개",
        frame.table_distance_m(),
        frame.mount_z() - crate::constants::table::SURFACE_Z,
        intercept.hit_planes().len(),
    ));

    if !parked {
        ui.colored_label(
            egui::Color32::from_rgb(220, 165, 80),
            "공 비행 중 — 주차 후 조정 가능",
        );
    }

    ui.add_space(4.0);
    let default_frame = defaults::rail_frame();
    let default_intercept = crate::robot::motion::InterceptWindow::default();
    let changed = *frame != default_frame || *intercept != default_intercept;
    ui.add_enabled_ui(parked && changed, |ui| {
        if ui.button("실측 기본값으로").clicked() {
            *frame = default_frame;
            *intercept = default_intercept;
        }
    });
}

/// 라이브 월드에서 시나리오 재실행 채점을 한 프레임 갱신한다.
pub fn tick_eval_live(ui_state: &mut PanelUiState, world: &SimWorld) {
    let Some(live) = ui_state.eval_live.as_mut() else {
        return;
    };
    if live.live_points.is_some() || live.net_passthrough {
        return;
    }
    if live.observer.observe(world) {
        if live.observer.net_passthrough {
            live.net_passthrough = true;
            return;
        }
        live.live_points = Some(live.observer.points());
    }
}

fn begin_eval_live_shot(
    ui_state: &mut PanelUiState,
    world: &Arc<Mutex<SimWorld>>,
    controls: &Arc<Mutex<SimRuntimeControls>>,
    shot_number: usize,
) {
    if !(1..=eval::TOTAL_SHOTS).contains(&shot_number) {
        return;
    }
    let settings = {
        let progress = ui_state.eval.lock().expect("eval progress");
        let Some(report) = progress.report.as_ref() else {
            return;
        };
        let Some(shot) = report.shots.get(shot_number - 1) else {
            return;
        };
        (shot.settings.clone(), shot.zone)
    };

    let Ok(world_guard) = world.lock() else {
        return;
    };
    let observer = eval::LiveObserver::new(&world_guard);
    drop(world_guard);

    ui_state.shooter = settings.0.clone();
    ui_state.eval_live = Some(EvalLiveRun {
        shot_number,
        zone: settings.1,
        observer,
        live_points: None,
        net_passthrough: false,
    });

    if let Ok(mut ctrl) = controls.lock() {
        ctrl.shooter = settings.0;
        ctrl.request_shoot();
    }
}

fn draw_view_panel(ui: &mut egui::Ui, ui_state: &mut PanelUiState) {
    egui::CollapsingHeader::new("Camera / time")
        .default_open(true)
        .show(ui, |ui| {
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
        });
    ui.collapsing("Swing", |ui| {
        ui.checkbox(
            &mut ui_state.use_bang_bang_swing,
            "Bang-bang (pure torque, debug)",
        );
        ui.small("commit을 quintic 대신 순수 토크 bang-bang으로 — 육안 비교용");
        ui.checkbox(
            &mut ui_state.use_fixed_swing_dictionary,
            "Fixed swing dictionary (debug, no IK)",
        );
        ui.small("commit을 IK 없는 고정 스윙 딕셔너리로 — 육안 비교용");
        ui.horizontal(|ui| {
            ui.label("임팩트 시각:");
            ui.radio_value(
                &mut ui_state.fixed_swing_impact_strategy,
                crate::robot::motion::ImpactTimeStrategy::Midpoint,
                "중간 시점",
            );
            ui.radio_value(
                &mut ui_state.fixed_swing_impact_strategy,
                crate::robot::motion::ImpactTimeStrategy::PeakRacketSpeed,
                "최대 속도 시점",
            );
        });
    });
    egui::CollapsingHeader::new("Debug overlays")
        .default_open(false)
        .show(ui, |ui| {
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
            });
            debug_checkbox(ui, &mut d.fail_status, "fail status", |ui| {
                ui.strong("실패 사유 (Status)");
                ui.label("스윙 포기·건너뛴 이유를 Status에 표시합니다.");
            });
            debug_checkbox(ui, &mut d.unreachable_x, "unreachable X", |ui| {
                ui.strong("도달 불가 목표");
                ui.label("한계에 걸린 목표점에 빨간 X를 그립니다.");
            });
            debug_checkbox(ui, &mut d.joint_limits, "joint limits", |ui| {
                ui.strong("관절 한계");
                ui.label("가동 범위 끝이면 링크를 빨갛게 표시합니다.");
            });
            debug_checkbox(ui, &mut d.torque_hud, "torque HUD", |ui| {
                ui.strong("토크·가속 경고 (Status)");
                ui.label("토크/가속 초과·관절 리밋·테이블 침투를 Status에 표시합니다.");
            });
            debug_checkbox(ui, &mut d.joint_anchors, "joint windows", |ui| {
                ui.strong("관절 앵커 창");
                ui.label("각 관절 옆에 각도·토크 HUD를 붙입니다.");
                ui.add_space(4.0);
                ui.strong("외곽선 색");
                ui.label("노랑 — 정상 (한계 안)");
                ui.label("주황 — 관절 가동범위 끝, 또는 토크 상한 초과");
            });
            debug_checkbox(ui, &mut d.commit_bar, "commit bar", |ui| {
                ui.strong("스윙 결정 타이밍");
                ui.label("tti가 commit 구간인지 Status 막대로 표시합니다.");
            });
            debug_checkbox(ui, &mut d.table_obb, "table OBB", |ui| {
                ui.strong("테이블 침투 OBB");
                ui.label("테이블을 뚫는 링크 box를 그립니다.");
            });
            debug_checkbox(ui, &mut d.net_gate, "net gate tone", |ui| {
                ui.strong("네트 게이트");
                ui.label("네트 미달 탄도면 공을 회색으로 바꿉니다.");
            });
            debug_checkbox(ui, &mut d.predicted_arc, "predicted arc", |ui| {
                ui.strong("예측 탄도");
                ui.label("예상 경로 (하늘색 점).");
            });
            debug_checkbox(ui, &mut d.truth_arc, "truth arc", |ui| {
                ui.strong("실제 탄도");
                ui.label("실제 경로 (주황 점).");
            });
            debug_checkbox(ui, &mut d.swing_ghost, "swing ghost", |ui| {
                ui.strong("스윙 경로");
                ui.label("확정 스윙의 라켓 중심 경로.");
            });
            debug_checkbox(ui, &mut d.rail_stroke, "rail stroke", |ui| {
                ui.strong("레일 이동 범위");
                ui.label("레일 양끝과 현재 위치.");
            });
            debug_checkbox(ui, &mut d.aim_band, "aim band", |ui| {
                ui.strong("Random 조준 대역");
                ui.label("Random이 겨냥하는 y≈0 구간.");
            });
            debug_checkbox(ui, &mut d.omega_arrow, "ω arrow", |ui| {
                ui.strong("스핀 방향");
                ui.label("공 각속도 화살표.");
            });
        });
}

fn draw_eval_status(
    ui: &mut egui::Ui,
    ui_state: &PanelUiState,
    start_live_shot: &mut Option<usize>,
) {
    let running = ui_state.eval_running.load(Ordering::Relaxed);
    let progress = ui_state.eval.lock().expect("eval progress").clone();

    if running {
        ui.ctx().request_repaint();
        let frac = if progress.total == 0 {
            0.0
        } else {
            progress.done as f32 / progress.total as f32
        };
        ui.add_space(6.0);
        ui.add(
            egui::ProgressBar::new(frac)
                .text(format!("{}/{}", progress.done, progress.total))
                .animate(true),
        );
        return;
    }

    if let Some(err) = progress.error.as_ref() {
        ui.add_space(4.0);
        ui.colored_label(egui::Color32::from_rgb(220, 80, 80), err);
        return;
    }

    let Some(report) = progress.report.as_ref() else {
        return;
    };

    ui.add_space(6.0);
    let pass = report.passed();
    let (verdict, color) = if pass {
        ("Pass", egui::Color32::from_rgb(70, 190, 110))
    } else {
        ("Fail", egui::Color32::from_rgb(220, 110, 80))
    };
    ui.horizontal(|ui| {
        ui.colored_label(color, egui::RichText::new(verdict).strong());
        ui.monospace(format!("{}/{}", report.total, eval::MAX_SCORE));
        ui.weak(format!("need >{}", eval::PASS_SCORE_EXCLUSIVE));
    });

    ui.add_space(2.0);
    egui::Grid::new("eval_score_grid")
        .num_columns(5)
        .spacing([10.0, 2.0])
        .show(ui, |ui| {
            ui.weak("");
            ui.weak("0");
            ui.weak("1");
            ui.weak("2");
            ui.weak("3");
            ui.end_row();

            ui.label("All");
            for c in report.counts {
                ui.monospace(format!("{c}"));
            }
            ui.end_row();

            for zone in eval::Zone::ALL {
                let z = report.zone_score(zone);
                ui.label(zone.label());
                for c in z.counts {
                    ui.monospace(format!("{c}"));
                }
                ui.end_row();
            }
        });

    ui.add_space(2.0);
    ui.weak(format!(
        "zone totals  R {} · C {} · L {}",
        report.zone_score(eval::Zone::Right).total,
        report.zone_score(eval::Zone::Center).total,
        report.zone_score(eval::Zone::Left).total,
    ));

    ui.add_space(6.0);
    ui.label("Re-run shot");
    ui.weak("1–30 · same scenario in the live sim");
    let selected = ui_state.eval_live.as_ref().map(|l| l.shot_number);
    egui::Grid::new("eval_shot_picker")
        .num_columns(10)
        .spacing([2.0, 2.0])
        .show(ui, |ui| {
            for n in 1..=eval::TOTAL_SHOTS {
                let shot = &report.shots[n - 1];
                let fill = points_color(shot.points);
                let mut button = egui::Button::new(format!("{n}")).fill(fill);
                if selected == Some(n) {
                    button = button.stroke(egui::Stroke::new(1.5, egui::Color32::WHITE));
                }
                if ui.add_sized([22.0, 20.0], button).clicked() {
                    *start_live_shot = Some(n);
                }
                if n % 10 == 0 {
                    ui.end_row();
                }
            }
        });

    if let Some(live) = ui_state.eval_live.as_ref() {
        ui.add_space(4.0);
        ui.ctx().request_repaint();
        if live.net_passthrough {
            ui.colored_label(
                egui::Color32::from_rgb(220, 120, 80),
                egui::RichText::new(format!(
                    "shot {} ({}) · 네트 투과(무효)",
                    live.shot_number,
                    live.zone.label()
                ))
                .strong(),
            );
        } else {
            match live.live_points {
                Some(pts) => {
                    ui.colored_label(
                        points_color(pts),
                        egui::RichText::new(format!(
                            "shot {} ({}) · {} pt{}",
                            live.shot_number,
                            live.zone.label(),
                            pts,
                            if pts == 1 { "" } else { "s" }
                        ))
                        .strong(),
                    );
                }
                None => {
                    ui.weak(format!(
                        "shot {} ({}) · running…",
                        live.shot_number,
                        live.zone.label()
                    ));
                }
            }
        }
    }
}

fn points_color(points: u8) -> egui::Color32 {
    return match points {
        0 => egui::Color32::from_rgb(70, 70, 75),
        1 => egui::Color32::from_rgb(90, 100, 140),
        2 => egui::Color32::from_rgb(60, 130, 160),
        _ => egui::Color32::from_rgb(50, 140, 90),
    };
}

fn start_eval_protocol(ui_state: &PanelUiState, world: &Arc<Mutex<SimWorld>>, mode: eval::Mode) {
    if ui_state
        .eval_running
        .compare_exchange(false, true, Ordering::SeqCst, Ordering::Relaxed)
        .is_err()
    {
        return;
    }
    let Ok(world_guard) = world.lock() else {
        ui_state.eval_running.store(false, Ordering::Relaxed);
        return;
    };
    let robot = Robot {
        arm: Arc::clone(&world_guard.arm),
        urdf: world_guard.urdf.clone(),
    };
    let physics = world_guard.physics;
    drop(world_guard);

    {
        let mut p = ui_state.eval.lock().expect("eval progress");
        *p = eval::Progress::default();
    }

    let progress = Arc::clone(&ui_state.eval);
    let running = Arc::clone(&ui_state.eval_running);
    let launch = ui_state.eval_launch;
    std::thread::spawn(move || {
        let _report =
            crate::sim::eval::Protocol::run(&robot, physics, &launch, mode, Some(progress));
        running.store(false, Ordering::Relaxed);
    });
}

/// 관절 한계[deg]가 있으면 그 범위로 자르는 `DragValue` — 한계 밖 값을 넣고
/// `move_to_fastest`가 "InfeasibleSwing"으로 조용히 실패하는(로봇이 왜 안
/// 움직이는지 모르는) 경우를 입력 단계에서 막는다.
fn add_bounded_drag_value(ui: &mut egui::Ui, value: &mut f64, limit_deg: Option<(f64, f64)>) {
    let mut drag = egui::DragValue::new(value).suffix("°").speed(1.0);
    if let Some((lo, hi)) = limit_deg {
        drag = drag.range(lo..=hi);
    }
    ui.add(drag);
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
        physics::BallState::Parked => "주차 (슈터에 대기)",
        physics::BallState::InFlight => "비행 중",
    };
    let swing_ko = if status.swing_committed && status.arrives_on_time == Some(false) {
        "선행 이동 중 — 현재 한계로는 정시 도착 어려움"
    } else if status.swing_committed {
        "확정 — 예측 타격점으로 이동 중"
    } else if status.swing_abandoned {
        "포기 — 이번 공은 안 침"
    } else {
        "대기"
    };

    egui::CollapsingHeader::new("Sim")
        .default_open(true)
        .show(ui, |ui| {
            ui.label(format!("시간  {:.2} s", status.sim_time));
        });

    egui::CollapsingHeader::new("Ball")
        .default_open(true)
        .show(ui, |ui| {
            ui.label(format!("상태  {ball_ko}"));
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
        });

    egui::CollapsingHeader::new("Robot")
        .default_open(true)
        .show(ui, |ui| {
            ui.label(format!("관절각 [rad]  {}", status.joints.join("  ")));
            ui.label(format!("스윙  {swing_ko}"));
            ui.label(format!("단계  {}", status.commit_phase.label_ko()));
        });

    egui::CollapsingHeader::new("Impact")
        .default_open(true)
        .show(ui, |ui| {
            if let Some(pred) = &status.debug_prediction {
                let p = pred.impact_position.coords;
                ui.label(format!("임팩트까지  {:.3} s", pred.time_to_impact_secs));
                ui.label(format!(
                    "예상 위치 [m]  x {:.2}  y {:.2}  z {:.2}",
                    p.x, p.y, p.z
                ));
            } else {
                ui.small("아직 예측 없음");
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
        });

    if debug.fail_status && (status.last_fail_text.is_some() || status.unreachable_xyz.is_some()) {
        egui::CollapsingHeader::new("Fail")
            .default_open(true)
            .show(ui, |ui| {
                if let Some(text) = &status.last_fail_text {
                    ui.colored_label(egui::Color32::from_rgb(255, 120, 90), text);
                }
                if let Some([x, y, z]) = status.unreachable_xyz {
                    ui.label(format!("목표점 [m]  x {x:.2}  y {y:.2}  z {z:.2}"));
                }
            });
    }

    if debug.torque_hud {
        let has_torque = !status.torque_peak_nm.is_empty() || !status.torque_now_nm.is_empty();
        let has_warn = status.accel_over
            || status.torque_over.iter().any(|&o| o)
            || status.joint_at_limit.iter().any(|&o| o)
            || status.table_pen_depth > 1e-4;
        if has_torque || has_warn {
            egui::CollapsingHeader::new("Limits")
                .default_open(true)
                .show(ui, |ui| {
                    if !status.torque_peak_nm.is_empty() {
                        let peaks: Vec<String> = status
                            .torque_peak_nm
                            .iter()
                            .enumerate()
                            .map(|(i, t)| format!("j{i}={t:.2}"))
                            .collect();
                        ui.label(format!("토크 peak  {}", peaks.join("  ")));
                    }
                    if !status.torque_now_nm.is_empty() {
                        let now: Vec<String> = status
                            .torque_now_nm
                            .iter()
                            .enumerate()
                            .map(|(i, t)| format!("j{i}={t:+.2}"))
                            .collect();
                        ui.label(format!("토크 now   {}", now.join("  ")));
                    }
                    if status.accel_over {
                        ui.colored_label(egui::Color32::YELLOW, "관절 가속이 허용 상한을 넘김");
                    }
                    if status.torque_over.iter().any(|&o| o) {
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
                        ui.colored_label(
                            egui::Color32::from_rgb(80, 220, 230),
                            format!("테이블 침투  {:.1} mm", status.table_pen_depth * 1000.0),
                        );
                    }
                });
        }
    }
}

fn draw_joint_anchor_windows(
    ctx: &egui::Context,
    status: &StatusSnapshot,
    screens: &[Option<egui::Pos2>],
) {
    let n = status
        .joint_world
        .len()
        .min(status.joint_q.len())
        .min(screens.len());

    // 투영점 → 라벨 위치. 가까우면 화면에서 밀어 겹침을 줄인다.
    const MIN_SEP: f32 = 78.0;
    let mut anchors: Vec<(usize, egui::Pos2)> = Vec::with_capacity(n);
    for i in 0..n {
        if let Some(pos) = screens[i] {
            anchors.push((i, pos));
        }
    }
    let mut label_pos: Vec<egui::Pos2> = anchors
        .iter()
        .map(|&(_, p)| p + egui::vec2(14.0, -10.0))
        .collect();
    for _ in 0..8 {
        for a in 0..label_pos.len() {
            for b in (a + 1)..label_pos.len() {
                let delta = label_pos[b] - label_pos[a];
                let dist = delta.length();
                if dist >= MIN_SEP || dist < 1e-3 {
                    continue;
                }
                let push = (MIN_SEP - dist) * 0.5;
                let dir = if dist < 1e-3 {
                    egui::vec2(0.0, 1.0)
                } else {
                    delta / dist
                };
                label_pos[a] -= dir * push;
                label_pos[b] += dir * push;
            }
        }
    }

    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("joint_anchor_guides"),
    ));

    for (slot, &(i, joint_pos)) in anchors.iter().enumerate() {
        let label = label_pos[slot];
        let q = status.joint_q[i];
        let q_min = status.joint_q_min.get(i).copied().flatten();
        let q_max = status.joint_q_max.get(i).copied().flatten();
        let tau = status.torque_now_nm.get(i).copied().unwrap_or(0.0);
        let tau_max = status.torque_limit_nm.get(i).copied().unwrap_or(6.0);
        let at_limit = status.joint_at_limit.get(i).copied().unwrap_or(false);
        let torque_hot =
            status.torque_over.get(i).copied().unwrap_or(false) || tau.abs() > tau_max + 1e-6;
        let accent = if at_limit || torque_hot {
            egui::Color32::from_rgb(255, 120, 80)
        } else {
            egui::Color32::from_rgb(255, 220, 90)
        };

        let area_id = egui::Id::new(("joint_anchor", i));

        // 관절 점 + 리더 라인 (실제 투영 위치 확인용)
        painter.circle_filled(joint_pos, 3.5, accent);
        painter.circle_stroke(
            joint_pos,
            5.0,
            egui::Stroke::new(
                1.0,
                egui::Color32::from_rgba_unmultiplied(255, 255, 255, 180),
            ),
        );
        painter.line_segment(
            [joint_pos, label],
            egui::Stroke::new(
                1.0,
                egui::Color32::from_rgba_unmultiplied(220, 220, 230, 180),
            ),
        );

        let frame = egui::Frame::NONE
            .fill(egui::Color32::from_rgba_unmultiplied(12, 14, 18, 200))
            .stroke(egui::Stroke::new(1.0, accent))
            .corner_radius(5.0)
            .inner_margin(egui::Margin::symmetric(6, 4));

        egui::Area::new(area_id)
            .fixed_pos(label)
            .order(egui::Order::Foreground)
            .sense(egui::Sense::hover())
            .show(ctx, |ui| {
                frame.show(ui, |ui| {
                    ui.set_max_width(128.0);
                    ui.strong(format!("j{i}"));
                    // 내부는 rad, HUD만 deg. 현재값 강조 + 범위는 … 로.
                    let q_deg = q.to_degrees();
                    ui.label(format!("q  {q_deg:.0}°"));
                    draw_range_bar(ui, q_min, q, q_max, at_limit);
                    ui.weak(match (q_min, q_max) {
                        (Some(lo), Some(hi)) => {
                            format!("{:.0}° … {:.0}°", lo.to_degrees(), hi.to_degrees())
                        }
                        (None, Some(hi)) => format!("−∞ … {:.0}°", hi.to_degrees()),
                        (Some(lo), None) => format!("{:.0}° … +∞", lo.to_degrees()),
                        (None, None) => "−∞ … +∞".into(),
                    });
                    ui.add_space(2.0);
                    ui.label(format!("τ  {tau:+.2} N·m"));
                    draw_signed_bar(ui, -tau_max, tau, tau_max, torque_hot);
                    ui.weak(format!("{:+.1} … {:+.1}", -tau_max, tau_max));
                });
            });
    }
}

fn draw_range_bar(ui: &mut egui::Ui, min: Option<f64>, cur: f64, max: Option<f64>, hot: bool) {
    let (Some(lo), Some(hi)) = (min, max) else {
        return;
    };
    if hi - lo < 1e-9 {
        return;
    }
    let t = ((cur - lo) / (hi - lo)).clamp(0.0, 1.0) as f32;
    let desired = egui::vec2(120.0, 6.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(
        rect,
        2.0,
        egui::Color32::from_rgba_unmultiplied(40, 44, 52, 200),
    );
    let fill = egui::Rect::from_min_size(rect.min, egui::vec2(rect.width() * t, rect.height()));
    painter.rect_filled(
        fill,
        2.0,
        if hot {
            egui::Color32::from_rgb(240, 100, 70)
        } else {
            egui::Color32::from_rgb(90, 180, 255)
        },
    );
    let x = rect.left() + rect.width() * t;
    painter.line_segment(
        [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
        egui::Stroke::new(1.5, egui::Color32::WHITE),
    );
}

fn draw_signed_bar(ui: &mut egui::Ui, min: f64, cur: f64, max: f64, hot: bool) {
    if max - min < 1e-9 {
        return;
    }
    let t = ((cur - min) / (max - min)).clamp(0.0, 1.0) as f32;
    let mid = ((0.0 - min) / (max - min)).clamp(0.0, 1.0) as f32;
    let desired = egui::vec2(120.0, 6.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(
        rect,
        2.0,
        egui::Color32::from_rgba_unmultiplied(40, 44, 52, 200),
    );
    let zero_x = rect.left() + rect.width() * mid;
    let cur_x = rect.left() + rect.width() * t;
    let (left, right) = if cur_x >= zero_x {
        (zero_x, cur_x)
    } else {
        (cur_x, zero_x)
    };
    painter.rect_filled(
        egui::Rect::from_min_max(
            egui::pos2(left, rect.top()),
            egui::pos2(right, rect.bottom()),
        ),
        2.0,
        if hot {
            egui::Color32::from_rgb(240, 180, 60)
        } else {
            egui::Color32::from_rgb(120, 220, 140)
        },
    );
    painter.line_segment(
        [
            egui::pos2(zero_x, rect.top() - 1.0),
            egui::pos2(zero_x, rect.bottom() + 1.0),
        ],
        egui::Stroke::new(1.0, egui::Color32::from_gray(180)),
    );
}

fn draw_commit_bar(ui: &mut egui::Ui, status: &StatusSnapshot) {
    let control = defaults::ControlParams::default();
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
