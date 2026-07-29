//! sim·실물 하드웨어 어댑터.

use crate::error::HwError;
use crate::robot;
use crate::swing;

pub mod dynamixel;
pub mod rail;
mod sim;

#[cfg(feature = "real")]
mod real;

pub use rail::AxlRail;
pub use sim::SimHardware;

#[cfg(feature = "real")]
pub use real::RealHardware;

/// 로봇 팔과 리니어 구동 인터페이스.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &swing::Trajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<robot::Pose, HwError>;
    fn is_busy(&mut self) -> bool {
        return false;
    }
}
