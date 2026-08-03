//! 패널 슬라이더·eval 상태.

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use super::super::debug::overlays::DebugOverlays;
use super::eval_live_run::EvalLiveRun;
use crate::constants::viewer::CAMERA_DIST_DEFAULT;
use crate::robot::RailFrame;
use crate::robot::motion::InterceptWindow;
use crate::sim::eval;
use crate::sim::launch;
use crate::sim::session::controls::SimRuntimeControls;

/// 패널 슬라이더 상태 — 매 프레임 `controls` 락 없이 UI를 그린다.
#[derive(Clone, Debug)]
pub struct PanelUiState {
    pub shooter: launch::Settings,
    /// 레일 마운트 설치 위치 ("Rig" 창). 공이 주차된 동안만 월드에 반영된다.
    pub rail_frame: RailFrame,
    /// 타격 후보 Y 창 ("Rig" 창). 주차 중에만 반영.
    pub intercept: InterceptWindow,
    pub time_scale: f64,
    /// OrbitCamera3d 거리 [m]
    pub camera_dist: f32,
    /// commit 시 quintic 대신 순수 토크 bang-bang을 쓸지 - 디버그 토글.
    pub use_bang_bang_swing: bool,
    /// commit 시 IK 없는 고정 스윙 딕셔너리를 쓸지 - 디버그 토글.
    pub use_fixed_swing_dictionary: bool,
    /// 고정 스윙 딕셔너리의 내부 임팩트 시각 전략 - 두 전략 비교용 선택기.
    pub fixed_swing_impact_strategy: crate::robot::motion::ImpactTimeStrategy,
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
    /// "Motor Test" 창 입력 — IK 없이 4관절 각 [deg]로 직접 지정하는 시작/끝 포즈.
    pub joint_test_start_deg: [f64; 4],
    pub joint_test_end_deg: [f64; 4],
    /// 마지막 Test 실행 결과 — 실패 사유 또는 계획된 소요 시간.
    pub joint_test_error: Option<String>,
    pub joint_test_last_duration_secs: Option<f64>,
}

impl PanelUiState {
    pub fn from_controls(controls: &SimRuntimeControls) -> Self {
        return Self {
            shooter: controls.shooter.clone(),
            rail_frame: controls.rail_frame,
            intercept: controls.intercept,
            time_scale: controls.time_scale,
            camera_dist: CAMERA_DIST_DEFAULT,
            use_bang_bang_swing: controls.use_bang_bang_swing,
            use_fixed_swing_dictionary: controls.use_fixed_swing_dictionary,
            fixed_swing_impact_strategy: controls.fixed_swing_impact_strategy,
            debug: DebugOverlays::debug_defaults(),
            eval: Arc::new(Mutex::new(eval::Progress::default())),
            eval_running: Arc::new(AtomicBool::new(false)),
            eval_mode: eval::Mode::Block,
            eval_launch: eval::LaunchParams::default(),
            eval_live: None,
            joint_test_start_deg: [0.0; 4],
            joint_test_end_deg: [0.0; 4],
            joint_test_error: None,
            joint_test_last_duration_secs: None,
        };
    }
}
