//! Windows 전용 실물 하드웨어 어댑터 골격.
//!
//! - 팔: Dynamixel → `rustypot` (양 OS, 2단계)
//! - 리니어: Ajinextek AXL → `libloading` + Windows DLL (plan §3.2)
//!
//! macOS 빌드에서는 `#[cfg(all(windows, feature = "real"))]`로 컴파일되지 않는다.

use pingpong_domain::{Hardware, HwError, Joints, SwingTrajectory};

/// Windows 실물 하드웨어 어댑터 (2단계).
pub struct RealHardware;

impl RealHardware {
    /// Dynamixel·AXL을 초기화한다.
    pub fn new() -> Result<Self, HwError> {
        todo!("실물 하드웨어 초기화 (rustypot + AXL, plan.md §3.2)")
    }
}

impl Hardware for RealHardware {
    fn command(&mut self, _trajectory: &SwingTrajectory) -> Result<(), HwError> {
        todo!("실물 하드웨어 스윙 명령")
    }

    fn read_joints(&mut self) -> Result<Joints, HwError> {
        todo!("실물 하드웨어 관절 읽기")
    }
}
