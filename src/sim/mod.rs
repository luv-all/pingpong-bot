//! Rapier3d 디지털 트윈 (plan §9).
//!
//! - [`physics`]: 탁구대·슈터·로봇 라켓·공
//! - [`session`]: 물리 스레드 + 공유 월드
//! - [`gui`]: kiss3d 3D + egui (feature `gui`)
//!   - 레이어 R/W: `BallHandle` / `RobotHandle` / `ShooterHandle`
//!   - 호스트: `run_scene_host` (table + optional layers)
//!   - 풀 패널: `run_sim_viewer`

pub mod eval_protocol;
pub mod gui;
pub mod physics;
pub mod session;

pub use eval_protocol::{
    EvalLaunchParams, EvalMode, EvalProgress, EvalProtocol, EvalReport, EvalShot, EvalZone,
    LiveShotObserver, MAX_SCORE, PASS_SCORE_EXCLUSIVE, SHOTS_PER_ZONE, TOTAL_SHOTS,
};
#[cfg(feature = "gui")]
pub use gui::{
    BallHandle, BallOnlyViewerOptions, BallVisual, RobotHandle, SceneHost, SceneHostOptions,
    SceneLayers, SceneLayersBuilder, SceneUiDraw, SceneUiHook, ShooterHandle, SimScene,
    SimSceneBuilder, SimViewer, SimViewerOptions, TableSceneOptions,
};
pub use gui::{CommitPhase, DebugOverlays, SimDebugSnapshot};
pub use physics::{ArmMultibody, BallShooterSettings, BallState, ShooterLayout, SimWorld};
pub use session::{SimBallEstimator, SimRuntimeControls, SimSession, SimSessionConfig};

// 하위 호환 모듈 경로 (`sim::world`, `sim::shooter`, …)
pub use gui::debug_overlays;
pub use gui::debug_snap;
pub use physics::arm_bodies;
pub use physics::shooter;
pub use physics::world;
pub use session::controls;
pub use session::estimator;
