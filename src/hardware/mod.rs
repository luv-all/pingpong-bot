//! sim·실물 하드웨어 어댑터.

use crate::error::HwError;
use crate::planner::SwingTrajectory;
use crate::robot::RobotPose;

mod sim;

#[cfg(all(windows, feature = "real"))]
mod real;

pub use sim::SimHardware;

#[cfg(all(windows, feature = "real"))]
pub use real::RealHardware;

/// 로봇 팔과 리니어 구동 인터페이스.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &SwingTrajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<RobotPose, HwError>;
    fn is_busy(&mut self) -> bool {
        return false;
    }
}
