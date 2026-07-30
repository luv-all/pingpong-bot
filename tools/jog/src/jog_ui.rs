//! egui SceneUiDraw 훅.

use std::sync::{Arc, Mutex};

use pingpong_bot::sim::gui::SceneUiDraw;

use crate::panel;
use crate::state::JogApp;

pub struct JogUi {
    pub app: Arc<Mutex<JogApp>>,
}

impl SceneUiDraw for JogUi {
    fn draw_ui(&mut self, ctx: &kiss3d::egui::Context) {
        if let Ok(mut app) = self.app.lock() {
            panel::draw(ctx, &mut app);
        }
    }
}
