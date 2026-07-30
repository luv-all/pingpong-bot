//! 조그 앱 상태.

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, ensure};
use pingpong_bot::hardware::{Hardware, RealHardware};
use pingpong_bot::robot::motion;
use pingpong_bot::robot::{self, Arm};
use pingpong_bot::sim::gui;
use pingpong_bot::sim::gui::ball;
use pingpong_bot::sim::gui::shooter;

use pingpong_bot::sim::launch;

use crate::plan::{self, Draft, Kind, SwingPlan};

use super::action::Action;
use super::phase::Phase;

pub struct JogApp {
    pub arm: Arc<Arm>,
    pub hardware: Arc<Mutex<RealHardware>>,
    pub robot: Option<gui::robot::Handle>,
    pub ball: Option<ball::Handle>,
    pub shooter: Option<shooter::Handle>,
    pub dry_run: bool,
    pub phase: Phase,
    /// Sync 시점 포즈 — 미리보기 시작점·Discard 복원.
    pub synced_pose: Option<robot::Pose>,
    /// 스테이징된 궤적 — 스윙은 [코스 추종 이동, 스윙] 두 개일 수 있다.
    pub staged: Vec<motion::Trajectory>,
    /// 아직 재생하지 않은 나머지 세그먼트 (앞 세그먼트가 끝나면 이어 재생).
    pending: Vec<motion::Trajectory>,
    pub duration_secs: f64,
    pub max_delta_deg: f64,
    pub draft: Draft,
    pub error: Option<String>,
    /// 스윙 계획 캐시 — planner가 로그를 뱉으므로 입력이 바뀔 때만 다시 푼다.
    swing_cache: Option<SwingCache>,
}

struct SwingCache {
    shooter: launch::Settings,
    start: robot::Pose,
    track_secs: f64,
    max_delta_deg: f64,
    result: Result<SwingPlan, String>,
}

impl JogApp {
    pub fn new(arm: Arc<Arm>, hardware: Arc<Mutex<RealHardware>>, dry_run: bool) -> Self {
        return Self {
            arm,
            hardware,
            robot: None,
            ball: None,
            shooter: None,
            dry_run,
            phase: Phase::NeedsSync,
            synced_pose: None,
            staged: Vec::new(),
            pending: Vec::new(),
            duration_secs: 1.0,
            max_delta_deg: 15.0,
            draft: Draft::default(),
            error: None,
            swing_cache: None,
        };
    }

    pub fn attach_robot(&mut self, robot: gui::robot::Handle) {
        self.robot = Some(robot);
    }

    pub fn attach_ball(&mut self, ball: ball::Handle) {
        self.ball = Some(ball);
    }

    pub fn attach_shooter(&mut self, shooter: shooter::Handle) {
        self.shooter = Some(shooter);
    }

    /// 패널의 슈터 값을 sim controls로 밀어 넣는다 (월드 슈터 자세·비주얼 갱신).
    pub fn push_shooter(&self) {
        if let Some(handle) = &self.shooter {
            handle.set_settings(self.draft.shooter.clone());
        }
    }

    /// 예측 도달점을 홀로그램 공에 반영. Swing 이외거나 예측 실패면 숨김.
    pub fn sync_ball_ghost(&self, preview: Option<&SwingPlan>) {
        let Some(ball) = &self.ball else {
            return;
        };
        let Some(preview) = preview.filter(|_| self.draft.kind == Kind::Swing) else {
            ball.set_position(None);
            ball.set_velocity(None);
            return;
        };
        let v = preview.prediction.incoming_velocity;
        ball.set_position(Some(preview.prediction.impact_position));
        ball.set_velocity(Some([v.x, v.y, v.z]));
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
        self.staged.clear();
        self.pending.clear();
        self.swing_cache = None;
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

    /// 입력(슈터 설정·동기화 포즈·시간·maxdelta)이 바뀌었을 때만 planner를 돌린다.
    ///
    /// 매 프레임 돌리면 `plan_best_swing`이 후보마다 뱉는 경고로 터미널이 잠긴다.
    pub fn refresh_swing_plan(&mut self) {
        if self.draft.kind != Kind::Swing {
            self.swing_cache = None;
            return;
        }
        let Some(start) = self.synced_pose.clone() else {
            self.swing_cache = None;
            return;
        };
        let fresh = self.swing_cache.as_ref().is_some_and(|c| {
            c.shooter == self.draft.shooter
                && c.start == start
                && c.track_secs == self.duration_secs
                && c.max_delta_deg == self.max_delta_deg
        });
        if fresh {
            return;
        }
        let result = plan::plan_swing(
            &self.arm,
            &start,
            &self.draft,
            self.duration_secs,
            self.max_delta_deg,
        )
        .map_err(|e| format!("{e:#}"));
        self.swing_cache = Some(SwingCache {
            shooter: self.draft.shooter.clone(),
            start,
            track_secs: self.duration_secs,
            max_delta_deg: self.max_delta_deg,
            result,
        });
    }

    pub fn swing_plan(&self) -> Option<&Result<SwingPlan, String>> {
        return self.swing_cache.as_ref().map(|c| &c.result);
    }

    /// 앞 세그먼트가 끝났으면 다음 세그먼트를 재생한다 (코스 추종 → 스윙).
    pub fn advance_segments(&mut self) {
        if self.pending.is_empty() || self.sim_busy() {
            return;
        }
        let next = self.pending.remove(0);
        if let Ok(robot) = self.robot() {
            robot.play(next);
        }
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
        self.staged.clear();
        self.pending.clear();
        self.phase = Phase::Ready;
        self.error = None;
        return Ok(());
    }

    pub fn apply(&mut self) -> Result<()> {
        ensure!(self.phase.can_apply(), "미리보기가 없습니다");
        ensure!(!self.staged.is_empty(), "staged trajectory 없음");
        let segments = self.staged.clone();
        {
            let mut hw = self.hardware.lock().expect("hardware");
            for traj in &segments {
                hw.command(traj).context("command")?;
                while hw.is_busy() {
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }
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
        let segments = if self.draft.kind == Kind::Swing {
            self.refresh_swing_plan();
            match self.swing_plan() {
                Some(Ok(plan)) => plan.segments.clone(),
                Some(Err(err)) => anyhow::bail!("{err}"),
                None => anyhow::bail!("스윙 계획 없음"),
            }
        } else {
            vec![plan::compose(
                &self.arm,
                &start,
                &self.draft,
                self.duration_secs,
                self.max_delta_deg,
            )?]
        };
        let robot = self.robot()?;
        robot.play(segments[0].clone());
        self.pending = segments[1..].to_vec();
        self.staged = segments;
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
