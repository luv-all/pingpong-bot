//! sim·실물 하드웨어 어댑터.

pub mod dynamixel;
mod hardware;
pub mod rail;
mod sim;

#[cfg(feature = "real")]
mod real;

pub use hardware::{AppliedRailRacketCommand, Hardware};
pub use rail::AxlRail;
pub use sim::SimHardware;

#[cfg(feature = "real")]
pub use real::RealHardware;
