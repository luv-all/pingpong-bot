//! egui 슈터·sim 제어 패널 (kiss3d `draw_ui` 오버레이).

use std::sync::{Arc, Mutex};

use kiss3d::egui;

use super::controls::SimRuntimeControls;
use super::shooter::{BallShooterSettings, BallState};
use super::world::SimWorld;

/// 패널 슬라이더 상태 — 매 프레임 `controls` 락 없이 UI를 그린다.
#[derive(Clone, Debug)]
pub struct PanelUiState {
    pub shooter: BallShooterSettings,
    pub time_scale: f64,
    /// OrbitCamera3d 거리 [m]
    pub camera_dist: f32,
}

impl PanelUiState {
    pub fn from_controls(controls: &SimRuntimeControls) -> Self {
        return Self {
            shooter: controls.shooter.clone(),
            time_scale: controls.time_scale,
            camera_dist: 4.5,
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
}

impl StatusSnapshot {
    /// 월드에서 한 프레임 분량의 상태를 읽는다.
    pub fn from_world(world: &SimWorld) -> Self {
        let bp = world.ball_position();
        let bv = world.ball_velocity();
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
        };
    }
}

/// 슈터 설정·상태 패널.
pub fn draw(
    ctx: &egui::Context,
    ui_state: &mut PanelUiState,
    controls: &Arc<Mutex<SimRuntimeControls>>,
    status: Option<&StatusSnapshot>,
) {
    let mut shoot = false;
    let mut park = false;

    egui::Window::new("pingpong-bot sim")
        .default_width(360.0)
        .default_pos([12.0, 12.0])
        .show(ctx, |ui| {
            ui.label("robot (y≈0) <- table -> shooter (+y) | Z-up: x=width, y=length, z=height");
            ui.separator();

            ui.collapsing("Aim (yaw / pitch / roll)", |ui| {
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
                let aim = ui_state.shooter.aim_direction();
                ui.label(format!("aim: ({:.2}, {:.2}, {:.2})", aim.x, aim.y, aim.z));
            });

            ui.collapsing("Muzzle (local)", |ui| {
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.lateral_offset_m, -0.5..=0.5)
                        .text("lateral [m]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.height_offset_m, -0.2..=0.3)
                        .text("height [m]"),
                );
            });

            ui.collapsing("Ball speed & spin", |ui| {
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.speed_mps, 3.0..=15.0)
                        .text("speed [m/s]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.topspin_rad_s, -80.0..=80.0)
                        .text("topspin [rad/s]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.sidespin_rad_s, -80.0..=80.0)
                        .text("sidespin [rad/s]"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.shooter.drill_spin_rad_s, -80.0..=80.0)
                        .text("drill [rad/s]"),
                );
            });

            ui.collapsing("Sim", |ui| {
                ui.add(
                    egui::Slider::new(&mut ui_state.time_scale, 0.1..=20.0)
                        .logarithmic(true)
                        .text("time scale"),
                );
                ui.add(
                    egui::Slider::new(&mut ui_state.camera_dist, 1.0..=12.0)
                        .text("camera zoom [m]"),
                );
                ui.label("3D: drag = orbit, scroll = zoom");
            });

            ui.horizontal(|ui| {
                if ui.button("Shoot").clicked() {
                    shoot = true;
                }
                if ui.button("Park ball").clicked() {
                    park = true;
                }
            });

            ui.separator();
            ui.heading("Status");
            if let Some(status) = status {
                let ball_state = match status.ball_state {
                    BallState::Parked => "parked",
                    BallState::InFlight => "in flight",
                };
                ui.label(format!("ball: {ball_state}"));
                ui.label(format!("sim time: {:.2} s", status.sim_time));
                ui.label(format!(
                    "pos: ({:.2}, {:.2}, {:.2}) m",
                    status.ball_pos.0, status.ball_pos.1, status.ball_pos.2
                ));
                ui.label(format!(
                    "vel: ({:.2}, {:.2}, {:.2}) m/s",
                    status.ball_vel.0, status.ball_vel.1, status.ball_vel.2
                ));
                ui.label(format!("joints [rad]: {:?}", status.joints));
            }
        });

    if let Ok(mut ctrl) = controls.try_lock() {
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
