//! 경량 호스트용 egui 드로어.

use std::sync::{Arc, Mutex};

/// 경량 호스트용 egui 드로어 (jog 패널 등). 풀 `enable_panel`과 상호 배타.
pub trait SceneUiDraw: Send {
    fn draw_ui(&mut self, ctx: &kiss3d::egui::Context);
}

/// [`SceneUiDraw`]를 호스트에 넘길 때 쓰는 공유 핸들.
pub type SceneUiHook = Arc<Mutex<dyn SceneUiDraw>>;
