//! Windows 전용 실물 하드웨어 어댑터 골격.
//!
//! - 팔: Dynamixel → `rustypot` (양 OS, 2단계)
//! - 리니어: Ajinextek AXL → `libloading` + Windows DLL (plan §3.2)

use crate::{Hardware, HwError, HwFailDetail, RobotPose, SwingTrajectory};

/// Windows 실물 하드웨어 어댑터 (Dynamixel + AXL — 미연결 시 NotImplemented).
pub struct RealHardware;

impl RealHardware {
    /// Dynamixel·AXL을 초기화한다.
    pub fn new() -> Result<Self, HwError> {
        return Err(HwError::ReadFailed {
            detail: HwFailDetail::NotImplemented,
        });
    }
}

impl Hardware for RealHardware {
    fn command(&mut self, trajectory: &SwingTrajectory) -> Result<(), HwError> {
        return Err(HwError::CommandFailed {
            duration_secs: trajectory.duration_secs,
            joint_count: trajectory.end_velocity.len(),
            detail: HwFailDetail::NotImplemented,
        });
    }

    fn read_pose(&mut self) -> Result<RobotPose, HwError> {
        return Err(HwError::ReadFailed {
            detail: HwFailDetail::NotImplemented,
        });
    }
}
