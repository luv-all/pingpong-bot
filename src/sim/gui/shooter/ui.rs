//! 슈터 파라미터 egui 위젯 — 메인 sim 패널과 jog가 공유한다.

use kiss3d::egui;

use crate::constants::table;
use crate::sim::launch;

/// 위젯 안 버튼 클릭 결과.
#[derive(Debug, Default, Clone, Copy)]
pub struct Buttons {
    pub shoot: bool,
    pub random: bool,
    pub park: bool,
}

/// 어떤 버튼을 그릴지 — 공을 실제로 쏘지 않는 툴(jog)은 Random만 쓴다.
#[derive(Debug, Clone, Copy)]
pub struct ButtonSet {
    pub shoot: bool,
    pub random: bool,
    pub park: bool,
}

impl ButtonSet {
    pub const ALL: Self = Self {
        shoot: true,
        random: true,
        park: true,
    };
    /// 발사 없이 파라미터만 굴리는 툴용.
    pub const RANDOM_ONLY: Self = Self {
        shoot: false,
        random: true,
        park: false,
    };
}

/// 슈터 파라미터 전체 + Shoot / Random / Park.
///
/// 슬라이더 범위가 여기 한 곳에만 있다 — 호출부가 늘어도 범위는 갈라지지 않는다.
pub fn draw(ui: &mut egui::Ui, settings: &mut launch::Settings, show: ButtonSet) -> Buttons {
    let mut buttons = Buttons::default();

    egui::CollapsingHeader::new("공 발사 위치")
        .default_open(true)
        .show(ui, |ui| {
            let muzzle = settings.muzzle_position();
            let mut x = f64::from(muzzle.x);
            let mut y = f64::from(muzzle.y);
            let mut z = f64::from(muzzle.z);
            let changed_x = ui
                .add(egui::Slider::new(&mut x, -0.50..=table::WIDTH_X + 0.50).text("X [m]"))
                .changed();
            let changed_y = ui
                .add(egui::Slider::new(&mut y, -0.50..=table::LENGTH_Y + 1.00).text("Y [m]"))
                .changed();
            let changed_z = ui
                .add(egui::Slider::new(&mut z, 0.05..=2.00).text("Z [m]"))
                .changed();
            if changed_x || changed_y || changed_z {
                settings.set_muzzle_xyz(x, y, z);
            }
            ui.small("탁구대 로봇 쪽 끝선=(0,0), Z=바닥 기준");
        });
    ui.collapsing("고급: 마운트 오프셋", |ui| {
        ui.add(egui::Slider::new(&mut settings.pos_offset_x_m, -0.8..=0.8).text("x [m]"));
        ui.add(egui::Slider::new(&mut settings.pos_offset_y_m, -0.6..=0.8).text("y [m]"));
        ui.add(egui::Slider::new(&mut settings.pos_offset_z_m, -0.3..=0.5).text("z [m]"));
        let m = settings.mount_position();
        ui.monospace(format!("mount {:.2} {:.2} {:.2}", m.x, m.y, m.z));
    });
    ui.collapsing("Aim", |ui| {
        let muzzle = settings.muzzle_position();
        let changed = ui
            .add(egui::Slider::new(&mut settings.yaw_deg, -25.0..=25.0).text("yaw [deg]"))
            .changed()
            | ui.add(egui::Slider::new(&mut settings.pitch_deg, -25.0..=25.0).text("pitch [deg]"))
                .changed()
            | ui.add(egui::Slider::new(&mut settings.roll_deg, -45.0..=45.0).text("roll [deg]"))
                .changed();
        if changed {
            settings.set_muzzle_position(muzzle);
        }
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
        if show.shoot && ui.button("Shoot").clicked() {
            buttons.shoot = true;
        }
        if show.random && ui.button("Random").clicked() {
            buttons.random = true;
        }
        if show.park && ui.button("Park").clicked() {
            buttons.park = true;
        }
    });

    return buttons;
}
