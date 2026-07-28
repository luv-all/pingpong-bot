//! 로봇 원시 R/W — jog 커맨드 enum 없음.

use std::sync::{Arc, Mutex};

use crate::SwingTrajectory;
use crate::robot::{Joints, RacketPose, RobotPose, RobotState};
use crate::sim::physics::world::SimWorld;

/// 로봇 원시 R/W — `SimWorld`의 [`RobotState`]에 위임.
///
/// jog 툴은 IK/궤적을 **밖에서** 만든 뒤 [`Self::play`] / [`Self::set_pose`]만 호출한다.
#[derive(Clone)]
pub struct RobotHandle {
    world: Arc<Mutex<SimWorld>>,
}

impl RobotHandle {
    pub fn new(world: Arc<Mutex<SimWorld>>) -> Self {
        return Self { world };
    }

    pub fn pose(&self) -> RobotPose {
        let world = self.world.lock().expect("sim 월드");
        let robot = world.robot();
        return RobotPose::new(robot.rail_x(), robot.joints().clone());
    }

    pub fn racket_pose(&self) -> Option<RacketPose> {
        let world = self.world.lock().expect("sim 월드");
        return world.robot().racket_pose(&world.arm);
    }

    pub fn is_busy(&self) -> bool {
        let world = self.world.lock().expect("sim 월드");
        return world.robot().is_swinging();
    }

    /// Sync용: 스윙 취소 후 관절·레일을 즉시 스냅.
    pub fn set_pose(&self, pose: RobotPose) {
        let mut world = self.world.lock().expect("sim 월드");
        *world.robot_mut() = RobotState::new(pose.joints, pose.rail_x);
    }

    /// 홀드 추종 목표 (스윙 중이 아닐 때 모터가 rate-limit으로 따라감).
    pub fn set_targets(&self, joints: Joints, rail_x: f64) {
        let mut world = self.world.lock().expect("sim 월드");
        let robot = world.robot_mut();
        robot.set_targets(joints);
        robot.set_rail_target(rail_x);
    }

    /// 궤적 미리보기 재생. 진행 중이면 교체(`replace_swing`).
    pub fn play(&self, trajectory: SwingTrajectory) {
        let mut world = self.world.lock().expect("sim 월드");
        world.robot_mut().replace_swing(trajectory);
    }

    pub fn cancel(&self) {
        let mut world = self.world.lock().expect("sim 월드");
        world.robot_mut().cancel_swing();
    }

    pub fn world(&self) -> Arc<Mutex<SimWorld>> {
        return Arc::clone(&self.world);
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::defaults;
    use crate::sim::physics::world::SimWorld;

    fn test_world() -> Arc<Mutex<SimWorld>> {
        let robot = defaults::robot().expect("robot");
        return Arc::new(Mutex::new(SimWorld::new(robot)));
    }

    #[test]
    fn robot_handle_set_pose_and_play_busy() {
        let world = test_world();
        let robot = RobotHandle::new(Arc::clone(&world));
        let pose = robot.pose();
        assert!(!robot.is_busy());

        let mut end = pose.joints.clone();
        if let Some(v) = end.values.get_mut(0) {
            *v += 0.1;
        }
        let traj = crate::SwingTrajectory::new(
            pose.joints.clone(),
            end,
            vec![0.0; pose.joints.values.len()],
            vec![0.0; pose.joints.values.len()],
            0.2,
            crate::RailMotion {
                start: pose.rail_x,
                end: pose.rail_x,
                start_velocity: 0.0,
                end_velocity: 0.0,
            },
        );
        robot.play(traj);
        assert!(robot.is_busy());
        robot.cancel();
        assert!(!robot.is_busy());

        let snapped = RobotPose::new(pose.rail_x + 0.01, pose.joints);
        robot.set_pose(snapped.clone());
        let got = robot.pose();
        assert!((got.rail_x - snapped.rail_x).abs() < 1e-9);
    }
}
