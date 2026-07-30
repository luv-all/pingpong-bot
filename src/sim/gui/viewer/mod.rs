//! kiss3d 3D + egui — Rapier sim 월드와 슈터 패널.

mod dynamic_nodes;
mod eval_live_run;
pub(crate) mod mesh_loader;
mod options;
mod panel;
mod panel_ui_state;
mod robot_render;
mod scene_dynamics;
mod status_snapshot;
mod urdf_visual_node;

pub use options::SimViewerOptions;
pub use panel_ui_state::PanelUiState;

use std::sync::{Mutex, MutexGuard, TryLockError};
use std::time::{Duration, Instant};

use crate::sim::physics::world::SimWorld;

/// 한 렌더 프레임이 월드 스냅샷을 기다려 주는 상한.
pub const WORLD_LOCK_WAIT: Duration = Duration::from_millis(2);

/// 재시도 간격 — 물리 스레드의 락 보유 시간(~100us)보다 충분히 짧게.
const WORLD_LOCK_RETRY: Duration = Duration::from_micros(50);

pub fn run(options: SimViewerOptions) -> Result<(), String> {
    return pollster::block_on(scene_dynamics::viewer_main(options));
}

/// 프레임 예산 안에서 짧게 기다렸다가 월드 락을 잡는다 (못 잡으면 `None`).
pub(crate) fn lock_world_for_frame(world: &Mutex<SimWorld>) -> Option<MutexGuard<'_, SimWorld>> {
    let deadline = Instant::now() + WORLD_LOCK_WAIT;
    loop {
        match world.try_lock() {
            Ok(guard) => return Some(guard),
            Err(TryLockError::Poisoned(_)) => return None,
            Err(TryLockError::WouldBlock) => {
                let now = Instant::now();
                if now >= deadline {
                    return None;
                }
                std::thread::sleep(WORLD_LOCK_RETRY.min(deadline - now));
            }
        }
    }
}
