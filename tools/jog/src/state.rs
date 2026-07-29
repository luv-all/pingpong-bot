//! Sync / Apply / Discard 상태머신 + 조그 앱 상태.

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, ensure};
use pingpong_bot::swing;
use pingpong_bot::{Arm, BallHandle, Hardware, Point3, RealHardware, RobotHandle, RobotPose};

use crate::motion::{self, MotionDraft, MotionKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Sync 필요 (시작 직후 / Apply 후).
    NeedsSync,
    /// 미리보기 1회 가능.
    Ready,
    /// 스테이징됨 — Apply 또는 Discard.
    Previewed,
    /// Apply 직후 — 수동 Sync 필수.
    AwaitingSync,
}

impl Phase {
    pub fn label(self) -> &'static str {
        return match self {
            Self::NeedsSync => "동기화 필요",
            Self::Ready => "준비",
            Self::Previewed => "미리보기",
            Self::AwaitingSync => "동기화 필요",
        };
    }

    pub fn can_preview(self) -> bool {
        return matches!(self, Self::Ready);
    }

    pub fn can_apply(self) -> bool {
        return matches!(self, Self::Previewed);
    }

    pub fn can_discard(self) -> bool {
        return matches!(self, Self::Previewed);
    }

    pub fn can_sync(self) -> bool {
        return true;
    }
}

pub struct JogApp {
    pub arm: Arc<Arm>,
    pub hardware: Arc<Mutex<RealHardware>>,
    pub robot: Option<RobotHandle>,
    pub ball: Option<BallHandle>,
    pub dry_run: bool,
    pub phase: Phase,
    /// Sync 시점 포즈 — 미리보기 시작점·Discard 복원.
    pub synced_pose: Option<RobotPose>,
    pub staged: Option<swing::Trajectory>,
    pub duration_secs: f64,
    pub max_delta_deg: f64,
    pub draft: MotionDraft,
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
            draft: MotionDraft::default(),
            error: None,
        };
    }

    pub fn attach_robot(&mut self, robot: RobotHandle) {
        self.robot = Some(robot);
    }

    pub fn attach_ball(&mut self, ball: BallHandle) {
        self.ball = Some(ball);
    }

    /// AimBall / SwingBall 도달점을 홀로그램 공에 반영. 그 외 모션은 숨김.
    pub fn sync_arrival_ghost(&self) {
        let Some(ball) = &self.ball else {
            return;
        };
        let show = matches!(self.draft.kind, MotionKind::AimBall | MotionKind::SwingBall);
        if show {
            let [x, y, z] = self.draft.arrival_xyz;
            ball.set_position(Some(Point3::new(x, y, z)));
            if self.draft.kind == MotionKind::SwingBall {
                ball.set_velocity(Some(self.draft.ball_vin));
            } else {
                ball.set_velocity(None);
            }
        } else {
            ball.set_position(None);
            ball.set_velocity(None);
        }
    }

    fn robot(&self) -> Result<&RobotHandle> {
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

    fn fill_draft_from_pose(&mut self, pose: &RobotPose) {
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
        let traj = motion::compose(
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

    pub fn live_pose(&self) -> Option<RobotPose> {
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

#[derive(Clone, Copy)]
pub enum Action {
    Sync,
    Discard,
    Apply,
    Preview,
}
