//! Rapier3d 디지털 트윈 (plan §9).
//!
//! - `SimWorld`: 탁구대·슈터(+x)·로봇 라켓(-x)·공
//! - `SimSession`: 물리 스레드 + 공유 월드
//! - `viewer`: kiss3d 3D + egui 슈터 패널 (feature `gui`)
//!
//! [`SimCamera`], [`SimHardware`], [`SimBallEstimator`]는 각각 `camera`, `hardware`, `estimator` 모듈에 있다.

mod ball_script;
pub(crate) mod controls;
mod estimator;
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

#[allow(unused_imports)]
pub use crate::camera::SimCamera;
#[allow(unused_imports)]
pub use crate::hardware::SimHardware;
pub use ball_script::{BallAction, BallEvent, BallScript, BallVec3};
pub use controls::{SimRuntimeControls, new_shutdown_flag};
pub use estimator::{SimBallEstimator, predict_impact};
pub use session::{SimSession, SimSessionConfig};
pub use shooter::{BallShooterSettings, BallState, ShooterLayout};
#[cfg(feature = "gui")]
pub use viewer::{SimViewerOptions, run as run_sim_viewer};
pub use world::SimWorld;
