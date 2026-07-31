//! Rapier sim `Hardware` 어댑터.
//!
//! `Hardware` 포트 구현 — 명령·관절 읽기는 domain `robot::State`에 위임하고,
//! Rapier collider 동기화는 물리 스레드(`SimWorld::step`)가 FK로 처리한다.

use crate::robot;
use std::sync::{Arc, Mutex};

use crate::error::HwError;
use crate::hardware::Hardware;
use crate::robot::motion;
use tracing::debug;

use crate::sim::physics::world::SimWorld;

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
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError> {
        {
            let mut world = self.world.lock().expect("sim 월드");
            // ground truth 모드에서는 물리 스레드만 타격
            if world.use_ground_truth() {
                debug!("ground truth 타격 모드 — control 스윙 명령 무시");
                return Ok(());
            }
            if world.robot().is_swinging() {
                debug!("sim 이미 스윙 중 — 제어 루프 명령 무시");
                return Ok(());
            }
            if world.swing_committed() {
                debug!("이번 공에 이미 스윙 commit — 재계획 무시");
                return Ok(());
            }
            let arm = Arc::clone(&world.arm);
            let return_pose =
                robot::Pose::new(world.robot().rail_x(), world.robot().joints().clone());
            world.robot_mut().set_auto_return_to_center(false);
            world
                .robot_mut()
                .replace_motion_and_return(trajectory.clone(), return_pose);
            world.mark_swing_committed();
            world.debug_snap_mut().set_committed_path(&arm, &trajectory);
        }
        self.command_count += 1;

        debug!(
            commands = self.command_count,
            duration_secs = trajectory.duration_secs,
            rail_start = trajectory.rail.start,
            rail_end = trajectory.rail.end,
            goal = ?trajectory.end.values,
            end_vel = ?trajectory.end_velocity,
            peak_speed = trajectory.peak_joint_speed(),
            peak_rail_speed = trajectory.peak_rail_speed(),
            "sim quintic 스윙 적용"
        );
        return Ok(());
    }

    fn read_pose(&mut self) -> Result<robot::Pose, HwError> {
        let world = self.world.lock().expect("sim 월드");
        let robot = world.robot();
        return Ok(robot::Pose::new(robot.rail_x(), robot.joints().clone()));
    }

    fn is_busy(&mut self) -> bool {
        let world = self.world.lock().expect("sim 월드");
        // ground truth 타격 중이면 control이 plan_swing을 돌리지 않게 한다
        return world.use_ground_truth() || world.swing_committed() || world.robot().is_swinging();
    }
}
