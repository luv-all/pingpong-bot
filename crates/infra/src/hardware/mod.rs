//! sim·실물 하드웨어 어댑터.

mod sim;

#[cfg(all(windows, feature = "real"))]
mod real;

pub use sim::SimHardware;

#[cfg(all(windows, feature = "real"))]
pub use real::RealHardware;
