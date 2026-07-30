//! 제어 워커 — 하드웨어 단독 소유자.
//!
//! `read_pose → plan_best → command` 세 단계가 전부 이 스레드 안에서만 일어난다.
//! [`tools/jog`]의 Apply가 "sync한 포즈로 만든 궤적을 그대로 보낸다"로 보장하는 것을, 여기서는
//! 포즈를 다른 스레드가 아예 볼 수 없다는 사실로 보장한다.
//!
//! 커밋은 **1회 래치**다. 단발이므로 성공하든 포기하든 루프를 빠져나온다.
//!
//! [`tools/jog`]: https://github.com/luv-all/pingpong-bot/tree/main/tools/jog

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::error::{DomainError, HwError, SwingPlanError};
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::motion::Planner;
use pingpong_bot::robot::{self, Arm};
use tracing::{debug, info, info_span, warn};

use super::fmt::{f2, f2_slice};
use super::{CommitRequest, PoseMsg, ShotEvent, SimUpdate, SwingMsg};

/// 예측의 `time_to_impact_secs`는 요청 시각 기준이다. 계획을 시작할 때 이미 이만큼 낡았으면
/// 그 예측으로 세운 궤적은 임팩트 시점이 어긋난다 — 버리고 다음 요청을 기다린다.
const MAX_REQUEST_AGE_SECS: f64 = 0.015;

/// 계획 재시도 스로틀. sim `SWING_RETRY_THROTTLE_SECS`와 같은 값이다.
/// 57600 baud에서 `read_pose`는 sync_read 왕복이라 매 프레임 때리면 안 된다.
const PLAN_THROTTLE_SECS: f64 = 0.020;

const BUSY_POLL: Duration = Duration::from_millis(5);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);

/// 제어 워커를 띄운다. 항상 마지막에 [`ShotEvent::Done`]을 보낸다.
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    home: bool,
    rx: Receiver<CommitRequest>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
) -> JoinHandle<()> {
    return thread::spawn(move || {
        let _span = info_span!("control").entered();

        if home && let Err(error) = move_to_center(hardware.as_mut(), &arm) {
            warn!(%error, "홈 이동 실패 — 현재 자세에서 시작한다");
        }

        match hardware.read_pose() {
            // armed 로그는 메인이 `ShotEvent::Armed`로 한 곳에서만 찍는다.
            Ok(pose) => {
                if let Some(sim_tx) = &sim_tx {
                    let _ = sim_tx.try_send(SimUpdate {
                        pose: Some(PoseMsg::from(&pose)),
                        ..SimUpdate::default()
                    });
                }
                let _ = event_tx.send(ShotEvent::Armed { pose });
            }
            Err(error) => {
                let _ = event_tx.send(ShotEvent::Failed {
                    reason: format!("시작 포즈 읽기 실패: {error}"),
                });
                let _ = event_tx.send(ShotEvent::Done);
                return;
            }
        }

        let committed = wait_for_commit(hardware.as_mut(), &arm, &rx, sim_tx.as_ref(), &event_tx);

        if committed {
            wait_idle(hardware.as_mut());
            if let Err(error) = move_to_center(hardware.as_mut(), &arm) {
                warn!(%error, "센터 복귀 실패");
            }
        }
        let _ = event_tx.send(ShotEvent::Done);
    });
}

/// 커밋할 때까지 요청을 처리한다. 반환 = 실제로 스윙을 보냈는가.
fn wait_for_commit(
    hardware: &mut dyn Hardware,
    arm: &Arm,
    rx: &Receiver<CommitRequest>,
    sim_tx: Option<&Sender<SimUpdate>>,
    event_tx: &Sender<ShotEvent>,
) -> bool {
    let mut last_attempt: Option<Instant> = None;
    let mut last_warn = Instant::now() - Duration::from_secs(10);
    let mut stale = 0_u64;

    loop {
        let request = match rx.recv_timeout(RECV_TIMEOUT) {
            Ok(request) => request,
            Err(RecvTimeoutError::Timeout) => continue,
            // 추정·카메라가 먼저 내려갔다 — 커밋 없이 끝.
            Err(RecvTimeoutError::Disconnected) => return false,
        };

        let age = request.age_secs();
        if age > MAX_REQUEST_AGE_SECS {
            stale += 1;
            if stale % 50 == 1 {
                warn!(age_secs = f2(age), stale, "커밋 요청이 낡음 — 버림");
            }
            continue;
        }
        if last_attempt.is_some_and(|at| at.elapsed().as_secs_f64() < PLAN_THROTTLE_SECS) {
            continue;
        }
        last_attempt = Some(Instant::now());

        let start = match hardware.read_pose() {
            Ok(pose) => pose,
            Err(error) => {
                warn!(%error, "포즈 읽기 실패 — 이번 계획 건너뜀");
                continue;
            }
        };

        match Planner::plan_best(arm, &request.predictions, &start) {
            Ok(planned) => {
                // 계획한 그 궤적을 그대로 보낸다 — 사이에 포즈를 다시 읽지 않는다.
                if let Err(error) = hardware.command(&planned.trajectory) {
                    let _ = event_tx.send(ShotEvent::Failed {
                        reason: format!("스윙 명령 실패: {error}"),
                    });
                    return false;
                }
                let trajectory = &planned.trajectory;
                debug!(
                    ball_y = f2(request.ball_y),
                    request_age_secs = f2(age),
                    "커밋 요청 소비"
                );
                // 커밋한 궤적을 sim 창이 그대로 재생하게 보낸다 (관전용).
                if let Some(sim_tx) = sim_tx {
                    let _ = sim_tx.try_send(SimUpdate {
                        swing: Some(SwingMsg::from_trajectory(trajectory)),
                        ..SimUpdate::default()
                    });
                }
                // 커밋 로그는 메인이 `ShotEvent::Committed` 필드로 한 곳에서만 찍는다.
                let _ = event_tx.send(ShotEvent::Committed {
                    time_to_impact_secs: planned.prediction.time_to_impact_secs,
                    duration_secs: trajectory.duration_secs,
                    impact: planned.prediction.impact_position,
                    rail_start: trajectory.rail.start,
                    rail_end: trajectory.rail.end,
                    peak_joint_speed: trajectory.peak_joint_speed(),
                });
                return true;
            }
            // 모터 보호 — sim과 같이 이번 공은 다시 시도하지 않는다.
            Err(DomainError::InfeasibleSwing(SwingPlanError::JointOrTorqueLimit {
                target_x,
                target_y,
                target_z,
            })) => {
                let _ = event_tx.send(ShotEvent::Infeasible {
                    reason: format!(
                        "토크·관절 한계 초과 (목표 x{} y{} z{})",
                        f2(target_x),
                        f2(target_y),
                        f2(target_z)
                    ),
                });
                return false;
            }
            // 이미 늦은 예측 — 로그 레벨만 낮춰 버린다.
            Err(DomainError::InfeasibleSwing(SwingPlanError::InsufficientTime {
                time_to_impact_secs,
                min_swing_secs,
            })) => debug!(
                time_to_impact_secs = f2(time_to_impact_secs),
                min_swing_secs = f2(min_swing_secs),
                "InsufficientTime — 창 재진입 대기"
            ),
            Err(error) => {
                // 계획 실패는 매 시도마다 debug로 남긴다 (원인 추적), warn은 1초 스로틀.
                debug!(
                    %error,
                    candidates = request.predictions.len(),
                    rail_x = f2(start.rail_x),
                    start_joints = f2_slice(&start.joints.values),
                    "스윙 계획 실패 — 재시도"
                );
                if last_warn.elapsed() >= Duration::from_secs(1) {
                    last_warn = Instant::now();
                    warn!(%error, "스윙 계획 실패");
                }
                let _ = event_tx.send(ShotEvent::PlanFailed {
                    reason: error.to_string(),
                });
            }
        }
    }
}

/// 센터(ready) 자세로 이동하고 완주까지 기다린다.
fn move_to_center(hardware: &mut dyn Hardware, arm: &Arm) -> Result<(), MoveError> {
    let start = hardware.read_pose().map_err(MoveError::Hardware)?;
    let trajectory = Planner::return_to_center(arm, &start).map_err(MoveError::Plan)?;
    log_home(&start, &trajectory);
    hardware.command(&trajectory).map_err(MoveError::Hardware)?;
    wait_idle(hardware);
    return Ok(());
}

fn log_home(start: &robot::Pose, trajectory: &pingpong_bot::robot::motion::Trajectory) {
    info!(
        from_rail_x = f2(start.rail_x),
        to_rail_x = f2(trajectory.follow_through_rail_x),
        duration_secs = f2(trajectory.duration_secs),
        target = f2_slice(&trajectory.end_joints().values),
        "real shot: 센터 이동 — 팔이 움직입니다"
    );
}

fn wait_idle(hardware: &mut dyn Hardware) {
    while hardware.is_busy() {
        thread::sleep(BUSY_POLL);
    }
}

#[derive(Debug)]
enum MoveError {
    Hardware(HwError),
    Plan(DomainError),
}

impl std::fmt::Display for MoveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::Hardware(error) => write!(f, "{error}"),
            Self::Plan(error) => write!(f, "{error}"),
        };
    }
}
