//! 제어 워커 — 하드웨어 단독 소유자.
//!
//! `read_pose → plan_best → command` 세 단계가 전부 이 스레드 안에서만 일어난다.
//! [`tools/jog`]의 Apply가 "sync한 포즈로 만든 궤적을 그대로 보낸다"로 보장하는 것을, 여기서는
//! 포즈를 다른 스레드가 아예 볼 수 없다는 사실로 보장한다.
//!
//! 연속 급구: `wait_for_commit → (idle) → return_to_center`를 바깥 루프로 반복한다.
//! `Infeasible`는 이번 스윙만 포기하고 루프는 계속한다.
//!
//! [`tools/jog`]: https://github.com/luv-all/pingpong-bot/tree/main/tools/jog

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::error::{DomainError, HwError, SwingPlanError};
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::motion::{self, Planner};
use pingpong_bot::robot::{self, Arm};
use tracing::{debug, info, info_span, warn};

use super::fmt::{f2, f2_slice};
use super::{CommitRequest, ControlStatus, PoseMsg, ShotEvent, Shutdown, SimUpdate, SwingMsg};

/// 예측의 `time_to_impact_secs`는 요청 시각 기준이다. 계획을 시작할 때 이미 이만큼 낡았으면
/// 그 예측으로 세운 궤적은 임팩트 시점이 어긋난다 — 버리고 다음 요청을 기다린다.
const MAX_REQUEST_AGE_SECS: f64 = 0.015;

/// 계획 재시도 스로틀. sim `SWING_RETRY_THROTTLE_SECS`와 같은 값이다.
/// 57600 baud에서 `read_pose`는 sync_read 왕복이라 매 프레임 때리면 안 된다.
const PLAN_THROTTLE_SECS: f64 = 0.020;

const BUSY_POLL: Duration = Duration::from_millis(5);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);

enum CommitOutcome {
    /// 커밋한 궤적 — 임팩트 시점 추종 오차를 재는 데 쓴다.
    Committed(Box<motion::Trajectory>),
    Infeasible,
    Disconnected,
    Failed,
}

/// 제어 워커를 띄운다. 셧다운·치명 실패 시 [`ShotEvent::Done`]을 보낸다.
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    home: bool,
    rx: Receiver<CommitRequest>,
    status_tx: Sender<ControlStatus>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
    return thread::spawn(move || {
        let _span = info_span!("control").entered();

        if home && let Err(error) = move_to_center(hardware.as_mut(), &arm) {
            warn!(%error, "홈 이동 실패 — 현재 자세에서 시작한다");
        }

        let mut shot_seq: u64 = 0;
        while !shutdown.is_down() {
            shot_seq = shot_seq.saturating_add(1);

            let pose = match hardware.read_pose() {
                Ok(pose) => pose,
                Err(error) => {
                    let _ = event_tx.send(ShotEvent::Failed {
                        shot_seq,
                        reason: format!("시작 포즈 읽기 실패: {error}"),
                    });
                    break;
                }
            };
            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    pose: Some(PoseMsg::from(&pose)),
                    ..SimUpdate::default()
                });
            }
            let _ = event_tx.send(ShotEvent::Armed { shot_seq, pose });
            let _ = status_tx.send(ControlStatus::Ready { shot_seq });

            let outcome = wait_for_commit(
                hardware.as_mut(),
                &arm,
                &rx,
                sim_tx.as_ref(),
                &event_tx,
                shot_seq,
                &shutdown,
            );

            match outcome {
                CommitOutcome::Failed | CommitOutcome::Disconnected => break,
                CommitOutcome::Committed(_) | CommitOutcome::Infeasible => {}
            }

            let _ = status_tx.send(ControlStatus::Recovering { shot_seq });

            // NOTE(결선): 진짜 랠리에서는 풀 센터 복귀 전에 다음 스윙을
            // 허용하도록 이 재무장 조건을 바꿀 수 있다. 지금은 연속 급구만.
            if let CommitOutcome::Committed(trajectory) = &outcome {
                measure_impact_tracking(hardware.as_mut(), trajectory);
                wait_idle(hardware.as_mut());
            }
            if let Err(error) = move_to_center(hardware.as_mut(), &arm) {
                warn!(%error, "센터 복귀 실패 — 현재 자세에서 Ready");
            }
            // 샷 N Attempt가 채널에 남아 샷 N+1에 쓰이지 않게 비운다.
            while rx.try_recv().is_ok() {}
        }
        let _ = event_tx.send(ShotEvent::Done);
    });
}

/// 커밋할 때까지 요청을 처리한다.
fn wait_for_commit(
    hardware: &mut dyn Hardware,
    arm: &Arm,
    rx: &Receiver<CommitRequest>,
    sim_tx: Option<&Sender<SimUpdate>>,
    event_tx: &Sender<ShotEvent>,
    shot_seq: u64,
    shutdown: &Shutdown,
) -> CommitOutcome {
    let mut last_attempt: Option<Instant> = None;
    let mut last_warn = Instant::now() - Duration::from_secs(10);
    let mut stale = 0_u64;

    loop {
        if shutdown.is_down() {
            return CommitOutcome::Disconnected;
        }
        let request = match rx.recv_timeout(RECV_TIMEOUT) {
            Ok(request) => request,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return CommitOutcome::Disconnected,
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
                if let Err(error) = hardware.command(&planned.trajectory) {
                    let _ = event_tx.send(ShotEvent::Failed {
                        shot_seq,
                        reason: format!("스윙 명령 실패: {error}"),
                    });
                    return CommitOutcome::Failed;
                }
                let trajectory = &planned.trajectory;
                debug!(
                    shot = shot_seq,
                    ball_y = f2(request.ball_y),
                    request_age_secs = f2(age),
                    "커밋 요청 소비"
                );
                if let Some(sim_tx) = sim_tx {
                    let _ = sim_tx.try_send(SimUpdate {
                        swing: Some(SwingMsg::from_trajectory(trajectory)),
                        ..SimUpdate::default()
                    });
                }
                let _ = event_tx.send(ShotEvent::Committed {
                    shot_seq,
                    time_to_impact_secs: planned.prediction.time_to_impact_secs,
                    duration_secs: trajectory.duration_secs,
                    impact: planned.prediction.impact_position,
                    rail_start: trajectory.rail.start,
                    rail_end: trajectory.rail.end,
                    peak_joint_speed: trajectory.peak_joint_speed(),
                });
                return CommitOutcome::Committed(Box::new(planned.trajectory));
            }
            // 이번 스윙만 포기 — 바깥 루프가 센터 복귀 후 다음 급구를 받는다.
            Err(DomainError::InfeasibleSwing(SwingPlanError::JointOrTorqueLimit {
                target_x,
                target_y,
                target_z,
            })) => {
                let _ = event_tx.send(ShotEvent::Infeasible {
                    shot_seq,
                    reason: format!(
                        "토크·관절 한계 초과 (목표 x{} y{} z{})",
                        f2(target_x),
                        f2(target_y),
                        f2(target_z)
                    ),
                });
                return CommitOutcome::Infeasible;
            }
            Err(DomainError::InfeasibleSwing(SwingPlanError::InsufficientTime {
                time_to_impact_secs,
                min_swing_secs,
            })) => debug!(
                time_to_impact_secs = f2(time_to_impact_secs),
                min_swing_secs = f2(min_swing_secs),
                "InsufficientTime — 창 재진입 대기"
            ),
            Err(error) => {
                debug!(
                    %error,
                    shot = shot_seq,
                    candidates = request.predictions.len(),
                    rail_x = f2(start.rail_x),
                    start_joints = f2_slice(&start.joints.values),
                    "스윙 계획 실패 — 재시도"
                );
                if last_warn.elapsed() >= Duration::from_secs(1) {
                    last_warn = Instant::now();
                    warn!(%error, shot = shot_seq, "스윙 계획 실패");
                }
                let _ = event_tx.send(ShotEvent::PlanFailed {
                    shot_seq,
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

/// 임팩트 순간 **명령각 대비 실제각** 오차를 잰다 [rad].
///
/// `JOINT_SPEED_DERATE`를 0.5 → 0.8로 올렸다. 한계를 실제보다 높이면 플래너가 모터가 못
/// 따라가는 궤적을 통과시키고, 정직한 거부 대신 **조용한 빗나감**이 된다 — 라켓이 늦게
/// 도착하는데 로그에는 성공으로 찍힌다. 그래서 실제로 따라오는지 여기서 확인한다.
///
/// **임팩트 시점에 딱 한 번만** 읽는다. 스윙 내내 폴링하면 `read_pose`(57600 baud sync_read)가
/// executor의 200 Hz 목표 전송과 버스를 다퉈, 재려던 추종 오차를 스스로 만들어낸다.
/// 한 번이면 그 간섭이 무시할 만하고, 정작 중요한 시점의 값을 얻는다.
fn measure_impact_tracking(hardware: &mut dyn Hardware, trajectory: &motion::Trajectory) {
    let started = Instant::now();
    let at_impact = Duration::from_secs_f64(trajectory.impact_time_secs);
    let Some(remaining) = at_impact.checked_sub(started.elapsed()) else {
        return;
    };
    thread::sleep(remaining);

    let elapsed = started.elapsed().as_secs_f64();
    let commanded = trajectory.sample_at(elapsed);
    let Ok(actual) = hardware.read_pose() else {
        return;
    };
    let worst = commanded
        .values
        .iter()
        .zip(actual.joints.values.iter())
        .map(|(c, a)| (c - a).abs())
        .fold(0.0_f64, f64::max);
    let per_joint: Vec<f64> = commanded
        .values
        .iter()
        .zip(actual.joints.values.iter())
        .map(|(c, a)| c - a)
        .collect();

    // sim 실측 기준은 0.002~0.003 rad (`docs/…/2026-07-27-return-power.md` §3.1).
    // 그보다 훨씬 크면 모터가 못 따라가는 것이고, 그 지점이 진짜 속도 한계다.
    info!(
        worst_rad = f2(worst),
        worst_deg = f2(worst.to_degrees()),
        error_rad = f2_slice(&per_joint),
        at_secs = f2(elapsed),
        "real shot: 임팩트 추종 오차 (sim 기준 0.002~0.003 rad)"
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
