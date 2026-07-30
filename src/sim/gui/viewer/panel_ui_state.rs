//! 패널 슬라이더·eval 상태.

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use super::super::debug::overlays::DebugOverlays;
use super::eval_live_run::EvalLiveRun;
use crate::constants::viewer::CAMERA_DIST_DEFAULT;
use crate::eval;
use crate::sim::launch;
use crate::sim::session::controls::SimRuntimeControls;

/// 패널 슬라이더 상태 — 매 프레임 `controls` 락 없이 UI를 그린다.
#[derive(Clone, Debug)]
pub struct PanelUiState {
    pub shooter: launch::Settings,
    pub time_scale: f64,
    /// OrbitCamera3d 거리 [m]
    pub camera_dist: f32,
    /// commit 시 quintic 대신 순수 토크 bang-bang을 쓸지 - 디버그 토글.
    pub use_bang_bang_swing: bool,
    pub debug: DebugOverlays,
    /// 평가 프로토콜 백그라운드 진행.
    pub eval: Arc<Mutex<eval::Progress>>,
    /// 평가 스레드가 돌아가는 중이면 true.
    pub eval_running: Arc<AtomicBool>,
    /// Block vs Alternating.
    pub eval_mode: eval::Mode,
    /// Eval 전용 발사 속도·좌우 각도 (Shooter 패널과 분리).
    pub eval_launch: eval::LaunchParams,
    /// Run 30 이후 선택한 시나리오를 시뮬에서 다시 실행 중일 때.
    pub eval_live: Option<EvalLiveRun>,
}

impl PanelUiState {
    pub fn from_controls(controls: &SimRuntimeControls) -> Self {
        return Self {
            shooter: controls.shooter.clone(),
            time_scale: controls.time_scale,
            camera_dist: CAMERA_DIST_DEFAULT,
            use_bang_bang_swing: controls.use_bang_bang_swing,
            debug: DebugOverlays::debug_defaults(),
            eval: Arc::new(Mutex::new(eval::Progress::default())),
            eval_running: Arc::new(AtomicBool::new(false)),
            eval_mode: eval::Mode::Block,
            eval_launch: eval::LaunchParams::default(),
            eval_live: None,
        };
    }
}
