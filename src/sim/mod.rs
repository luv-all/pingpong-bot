//! Rapier3d 디지털 트윈 (plan §9).
//!
//! - `SimWorld`: 탁구대·슈터(+x)·로봇 라켓(-x)·공
//! - `SimSession`: 물리 스레드 + 공유 월드
//! - `viewer`: kiss3d 3D + egui 슈터 패널 (feature `gui`)

mod estimator;
pub mod arm_bodies;
pub(crate) mod controls;
pub(crate) mod debug_overlays;
pub(crate) mod debug_snap;
#[cfg(feature = "gui")]
mod mesh_loader;
#[cfg(feature = "gui")]
mod panel;
mod rapier_convert;
pub(crate) mod session;
pub(crate) mod shooter;
#[cfg(feature = "gui")]
mod viewer;
pub(crate) mod world;

pub use arm_bodies::ArmMultibody;
pub use controls::{SimRuntimeControls, new_shutdown_flag};
pub use debug_overlays::DebugOverlays;
pub use debug_snap::{CommitPhase, SimDebugSnapshot};
pub use estimator::{SimBallEstimator, predict_impact};
pub use session::{SimSession, SimSessionConfig};
pub use shooter::{BallShooterSettings, BallState, ShooterLayout};
#[cfg(feature = "gui")]
pub use viewer::{SimViewerOptions, run as run_sim_viewer};
pub use world::SimWorld;
