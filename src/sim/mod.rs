//! Rapier3d 디지털 트윈 (plan §9).
//!
//! - [`physics`]: 탁구대·슈터·로봇 라켓·공
//! - [`session`]: 물리 스레드 + 공유 월드
//! - [`gui`]: kiss3d 3D + egui 슈터 패널 (feature `gui`)

pub mod gui;
pub mod physics;
pub mod session;

pub use gui::{CommitPhase, DebugOverlays, SimDebugSnapshot};
#[cfg(feature = "gui")]
pub use gui::{SimViewerOptions, run_sim_viewer};
pub use physics::{ArmMultibody, BallShooterSettings, BallState, ShooterLayout, SimWorld};
pub use session::{
    SimBallEstimator, SimRuntimeControls, SimSession, SimSessionConfig, new_shutdown_flag,
    predict_impact,
};

// 하위 호환 모듈 경로 (`sim::world`, `sim::shooter`, …)
pub use gui::debug_overlays;
pub use gui::debug_snap;
pub use physics::arm_bodies;
pub use physics::shooter;
pub use physics::world;
pub use session::controls;
pub use session::estimator;
