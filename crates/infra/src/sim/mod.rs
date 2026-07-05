//! Rapier3d 디지털 트윈 (plan §9).
//!
//! - `SimWorld`: 탁구대·슈터(+x)·로봇 라켓(-x)·공
//! - `SimCamera`: 3D 공 위치 → 핀홀 픽셀 투영
//! - `SimHardware`: `Hardware` 포트 — 관절 목표 적용
//! - `SimSession`: 물리 스레드 + 공유 월드
//! - `viewer`: kiss3d 3D + egui 슈터 패널 (feature `gui`)

mod camera;
mod controls;
mod hardware;
#[cfg(feature = "gui")]
#[cfg(feature = "gui")]
mod mesh_loader;
mod panel;
mod projection;
mod rapier_convert;
mod session;
mod shooter;
#[cfg(feature = "gui")]
mod viewer;
mod world;

pub use camera::SimCamera;
pub use controls::{SimRuntimeControls, new_shutdown_flag};
pub use hardware::SimHardware;
pub use session::{SimSession, SimSessionConfig};
pub use shooter::{BallShooterSettings, BallState, ShooterLayout};
pub use world::SimWorld;
#[cfg(feature = "gui")]
pub use viewer::{SimViewerOptions, run as run_sim_viewer};
