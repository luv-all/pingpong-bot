//! 조그 앱 상태.

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, ensure};
use pingpong_bot::Point3;
use pingpong_bot::hardware::{Hardware, RealHardware};
use pingpong_bot::robot::motion;
use pingpong_bot::robot::{self, Arm};
use pingpong_bot::sim::gui;
use pingpong_bot::sim::gui::ball;

use crate::plan::{self, Draft, Kind};

use super::action::Action;
use super::phase::Phase;

pub struct JogApp {
    pub arm: Arc<Arm>,
    pub hardware: Arc<Mutex<RealHardware>>,
    pub robot: Option<gui::robot::Handle>,
    pub ball: Option<ball::Handle>,
    pub dry_run: bool,
    pub phase: Phase,
    /// Sync 시점 포즈 — 미리보기 시작점·Discard 복원.
    pub synced_pose: Option<robot::Pose>,
    pub staged: Option<motion::Trajectory>,
    pub duration_secs: f64,
    pub max_delta_deg: f64,
    pub draft: Draft,
    pub error: Option<String>,
}

impl JogApp {
    pub fn new(arm: Arc<Arm>, hardware: Arc<Mutex<RealHardware>>, dry_run: bool) -> Self {
        return Self {
            arm,
            hardware,
            robot: None,
            ball: None,
            dry_run,
            phase: Phase::NeedsSync,
            synced_pose: None,
            staged: None,
            duration_secs: 1.0,
            max_delta_deg: 15.0,
            draft: Draft::default(),
            error: None,
        };
    }

    pub fn attach_robot(&mut self, robot: gui::robot::Handle) {
        self.robot = Some(robot);
    }

    pub fn attach_ball(&mut self, ball: ball::Handle) {
        self.ball = Some(ball);
    }

    /// AimBall / SwingBall 도달점을 홀로그램 공에 반영. 그 외 모션은 숨김.
    pub fn sync_arrival_ghost(&self) {
        let Some(ball) = &self.ball else {
            return;
        };
        let show = matches!(self.draft.kind, Kind::AimBall | Kind::SwingBall);
        if show {
            let [x, y, z] = self.draft.arrival_xyz;
            ball.set_position(Some(Point3::new(x, y, z)));
            if self.draft.kind == Kind::SwingBall {
                ball.set_velocity(Some(self.draft.ball_vin));
            } else {
                ball.set_velocity(None);
            }
        } else {
            ball.set_position(None);
            ball.set_velocity(None);
        }
    }

    fn robot(&self) -> Result<&gui::robot::Handle> {
        return self
            .robot
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("robot handle 없음"));
    }

    pub fn sync(&mut self) -> Result<()> {
        let pose = {
            let mut hw = self.hardware.lock().expect("hardware");
            hw.read_pose().context("read_pose")?
        };
        let robot = self.robot()?;
        robot.cancel();
        robot.set_pose(pose.clone());
        self.fill_draft_from_pose(&pose);
        self.synced_pose = Some(pose);
        self.staged = None;
        self.phase = Phase::Ready;
        self.error = None;
        return Ok(());
    }

    fn fill_draft_from_pose(&mut self, pose: &robot::Pose) {
        for (i, rad) in pose.joints.values.iter().enumerate() {
            if let Some(slot) = self.draft.angles_deg.get_mut(i) {
                *slot = rad.to_degrees();
            }
        }
        if let Some(rad) = pose.joints.values.get(self.draft.joint_index) {
            self.draft.joint_deg = rad.to_degrees();
        }
        self.draft.rail_x = pose.rail_x;
        self.draft.reach_dxyz = [0.0; 3];
        self.draft.tilt_pitch_deg = 0.0;
        self.draft.tilt_yaw_deg = 0.0;
    }

    pub fn discard(&mut self) -> Result<()> {
        ensure!(self.phase.can_discard(), "미리보기가 없습니다");
        let pose = self
            .synced_pose
            .clone()
            .ok_or_else(|| anyhow::anyhow!("synced pose 없음"))?;
        let robot = self.robot()?;
        robot.cancel();
        robot.set_pose(pose);
        self.staged = None;
        self.phase = Phase::Ready;
        self.error = None;
        return Ok(());
    }

    pub fn apply(&mut self) -> Result<()> {
        ensure!(self.phase.can_apply(), "미리보기가 없습니다");
        let traj = self
            .staged
            .clone()
            .ok_or_else(|| anyhow::anyhow!("staged trajectory 없음"))?;
        {
            let mut hw = self.hardware.lock().expect("hardware");
            hw.command(&traj).context("command")?;
            while hw.is_busy() {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        }
        self.phase = Phase::AwaitingSync;
        self.error = None;
        return Ok(());
    }

    pub fn preview_from_draft(&mut self) -> Result<()> {
        ensure!(self.phase.can_preview(), "먼저 동기화하세요");
        let start = self
            .synced_pose
            .clone()
            .ok_or_else(|| anyhow::anyhow!("synced pose 없음"))?;
        let traj = plan::compose(
            &self.arm,
            &start,
            &self.draft,
            self.duration_secs,
            self.max_delta_deg,
        )?;
        let robot = self.robot()?;
        robot.play(traj.clone());
        self.staged = Some(traj);
        self.phase = Phase::Previewed;
        self.error = None;
        return Ok(());
    }

    pub fn set_error(&mut self, err: impl ToString) {
        self.error = Some(err.to_string());
    }

    pub fn live_pose(&self) -> Option<robot::Pose> {
        return self.robot.as_ref().map(|r| r.pose());
    }

    pub fn sim_busy(&self) -> bool {
        return self.robot.as_ref().is_some_and(|r| r.is_busy());
    }
}

/// 패널에서 버튼 클릭 결과를 반영.
pub fn try_action(app: &mut JogApp, action: Action) {
    let result = match action {
        Action::Sync => app.sync(),
        Action::Discard => app.discard(),
        Action::Apply => app.apply(),
        Action::Preview => app.preview_from_draft(),
    };
    if let Err(err) = result {
        app.set_error(format!("{err:#}"));
    }
}
