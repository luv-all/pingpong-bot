//! Rapier sim `Hardware` 어댑터.
//!
//! `Hardware` 포트 구현 — 명령·관절 읽기는 domain `RobotState`에 위임하고,
//! Rapier collider 동기화는 물리 스레드(`SimWorld::step`)가 FK로 처리한다.

use std::sync::{Arc, Mutex};

use pingpong_domain::{Hardware, HwError, Joints, SwingTrajectory};
use tracing::debug;

use super::world::SimWorld;

/// Rapier sim용 `Hardware` 어댑터.
pub struct SimHardware {
    /// 공유 sim 월드
    world: Arc<Mutex<SimWorld>>,
    /// 누적 명령 횟수 (테스트·로그용)
    command_count: u64,
}

impl SimHardware {
    /// 월드 핸들로 어댑터를 만든다.
    pub fn new(world: Arc<Mutex<SimWorld>>) -> Self {
        return Self {
            world,
            command_count: 0,
        };
    }

    /// 지금까지 받은 스윙 명령 수.
    pub fn command_count(&self) -> u64 {
        return self.command_count;
    }
}

impl Hardware for SimHardware {
    fn command(&mut self, trajectory: &SwingTrajectory) -> Result<(), HwError> {
        self.command_count += 1;
        self.world
            .lock()
            .expect("sim 월드")
            .robot_mut()
            .set_targets_from_trajectory(trajectory);

        debug!(
            commands = self.command_count,
            duration_secs = trajectory.duration_secs,
            joints = ?trajectory.joints.values,
            "sim 라켓 궤적 적용"
        );
        return Ok(());
    }

    fn read_joints(&mut self) -> Result<Joints, HwError> {
        return Ok(self
            .world
            .lock()
            .expect("sim 월드")
            .robot()
            .joints()
            .clone());
    }
}
