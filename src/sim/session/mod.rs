//! Sim 런타임 — 세션 스레드·컨트롤·공 추정.

mod clock_handle;
mod config;
pub mod controls;
pub mod predict;
mod session;

pub use config::SimSessionConfig;
pub use controls::SimRuntimeControls;
pub(crate) use predict::predict_impact;
pub use session::SimSession;

pub(crate) use clock_handle::SimClockHandle;
