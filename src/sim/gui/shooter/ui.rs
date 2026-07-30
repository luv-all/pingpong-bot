//! 슈터 파라미터 egui 위젯 — 메인 sim 패널과 jog가 공유한다.

use kiss3d::egui;

use crate::sim::launch;

/// 위젯 안 버튼 클릭 결과.
#[derive(Debug, Default, Clone, Copy)]
pub struct Buttons {
    pub shoot: bool,
    pub random: bool,
    pub park: bool,
}

/// 슈터 파라미터 전체 + Shoot / Random / Park.
///
/// 슬라이더 범위가 여기 한 곳에만 있다 — 호출부가 늘어도 범위는 갈라지지 않는다.
pub fn draw(ui: &mut egui::Ui, settings: &mut launch::Settings) -> Buttons {
    let mut buttons = Buttons::default();

    ui.collapsing("Position", |ui| {
        ui.add(egui::Slider::new(&mut settings.pos_offset_x_m, -0.8..=0.8).text("x [m]"));
        ui.add(egui::Slider::new(&mut settings.pos_offset_y_m, -0.6..=0.8).text("y [m]"));
        ui.add(egui::Slider::new(&mut settings.pos_offset_z_m, -0.3..=0.5).text("z [m]"));
        let m = settings.mount_position();
        ui.monospace(format!("mount {:.2} {:.2} {:.2}", m.x, m.y, m.z));
    });
    ui.collapsing("Aim", |ui| {
        ui.add(egui::Slider::new(&mut settings.yaw_deg, -25.0..=25.0).text("yaw [deg]"));
        ui.add(egui::Slider::new(&mut settings.pitch_deg, -25.0..=25.0).text("pitch [deg]"));
        ui.add(egui::Slider::new(&mut settings.roll_deg, -45.0..=45.0).text("roll [deg]"));
    });
    ui.collapsing("Muzzle", |ui| {
        ui.add(egui::Slider::new(&mut settings.lateral_offset_m, -0.5..=0.5).text("lateral [m]"));
        ui.add(egui::Slider::new(&mut settings.height_offset_m, -0.2..=0.4).text("height [m]"));
    });
    ui.collapsing("Speed / spin", |ui| {
        ui.add(egui::Slider::new(&mut settings.speed_mps, 3.0..=15.0).text("speed [m/s]"));
        ui.add(egui::Slider::new(&mut settings.topspin_rad_s, -80.0..=80.0).text("topspin"));
        ui.add(egui::Slider::new(&mut settings.sidespin_rad_s, -80.0..=80.0).text("sidespin"));
        ui.add(egui::Slider::new(&mut settings.drill_spin_rad_s, -80.0..=80.0).text("drill"));
    });
    ui.horizontal(|ui| {
        if ui.button("Shoot").clicked() {
            buttons.shoot = true;
        }
        if ui.button("Random").clicked() {
            buttons.random = true;
        }
        if ui.button("Park").clicked() {
            buttons.park = true;
        }
    });

    return buttons;
}
