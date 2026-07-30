use crate::error::HwError;
use crate::motion;
use crate::robot;

/// 로봇 팔과 리니어 구동 인터페이스.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<robot::Pose, HwError>;
    fn is_busy(&mut self) -> bool {
        return false;
    }
}
