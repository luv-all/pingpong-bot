//! Sim 런타임 — 세션 스레드·컨트롤·공 추정.

pub mod controls;
pub mod estimator;
mod run;

pub use controls::SimRuntimeControls;
pub use estimator::SimBallEstimator;
pub(crate) use estimator::predict_impact;
pub use run::{SimSession, SimSessionConfig};

pub(crate) use run::SimClockHandle;
