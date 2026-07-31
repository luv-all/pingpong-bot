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
use pingpong_bot::error::{DomainError, HwError};
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::control::{
    HitTargetSelector, PositionControlError, PositionController, PredictionStage,
};
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
    /// 선추종으로 팔을 옮겨 뒀는데 공 신호가 끊겼다 — 스윙 여부와 무관하게 홈으로.
    BallGone,
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
            let _ = event_tx.send(ShotEvent::Armed {
                shot_seq,
                pose: pose.clone(),
            });
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
                CommitOutcome::Committed(_)
                | CommitOutcome::Infeasible
                | CommitOutcome::BallGone => {}
            }

            let _ = status_tx.send(ControlStatus::Recovering { shot_seq });

            // 목표 시각까지 대기한 뒤 샷 시작 자세로 복귀해 다음 공을 받는다.
            if let CommitOutcome::Committed(trajectory) = &outcome {
                measure_impact_tracking(hardware.as_mut(), trajectory);
                wait_idle(hardware.as_mut());
            }
            if let Err(error) = move_to_pose(hardware.as_mut(), &arm, &pose) {
                warn!(%error, "출발 자세 복귀 실패 — 현재 자세에서 Ready");
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
    let mut stale = 0_u64;
    let mut provisional_sent = false;
    let mut last_signal: Option<Instant> = None;
    let window = motion::InterceptWindow::default();
    let selector = HitTargetSelector::new(window.y_min, window.y_max).expect("기본 목표 선택 구간");

    loop {
        if shutdown.is_down() {
            return CommitOutcome::Disconnected;
        }
        let mut request = match rx.recv_timeout(RECV_TIMEOUT) {
            Ok(request) => request,
            Err(RecvTimeoutError::Disconnected) => return CommitOutcome::Disconnected,
            Err(RecvTimeoutError::Timeout) => {
                if provisional_sent
                    && last_signal.is_some_and(|at| at.elapsed().as_secs_f64() >= 0.50)
                {
                    info!("real shot: 정밀 예측 전 공 신호 끊김 — 출발 자세로 복귀");
                    return CommitOutcome::BallGone;
                }
                continue;
            }
        };
        last_signal = Some(Instant::now());
        // 명령을 보내기 전에는 아직 커밋하지 않았으므로 대기열의 예전
        // 목표를 버리고 가장 새 궤적으로 계획한다.
        while let Ok(newer) = rx.try_recv() {
            request = newer;
        }

        // 1차 목표로 이미 이동 중이면 다음 1차 프레임으로 계속 흔들지
        // 않는다. 오직 정밀 단계로 올라갔을 때만 진행 중 목표를 교체한다.
        if provisional_sent && request.stage == PredictionStage::Provisional {
            continue;
        }

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

        let plan_started = Instant::now();
        match PositionController::plan_best(arm, &start, &request.trajectory, &selector) {
            Ok(planned) => {
                let planning_secs = plan_started.elapsed().as_secs_f64();
                // 선추종 이동이 아직 돌고 있으면 여기서 끊는다. `command`는 busy면
                // **조용히 무시하고 `Ok`를 돌려주므로**, 안 끊으면 스윙이 통째로 사라진다.
                if hardware.is_busy() {
                    hardware.cancel();
                    wait_idle(hardware);
                }
                if let Err(error) = hardware.command(&planned.trajectory) {
                    let _ = event_tx.send(ShotEvent::Failed {
                        shot_seq,
                        reason: format!("목표 위치 이동 명령 실패: {error}"),
                    });
                    return CommitOutcome::Failed;
                }
                let trajectory = &planned.trajectory;
                debug!(
                    shot = shot_seq,
                    stage = ?request.stage,
                    ball_y = f2(request.ball_y),
                    request_age_secs = f2(age),
                    target_x = f2(planned.target.position.x),
                    target_y = f2(planned.target.position.y),
                    target_z = f2(planned.target.position.z),
                    "최적 목표 위치 요청 소비"
                );
                if let Some(sim_tx) = sim_tx {
                    let _ = sim_tx.try_send(SimUpdate {
                        swing: Some(SwingMsg::from_trajectory(trajectory)),
                        ..SimUpdate::default()
                    });
                }
                if request.stage == PredictionStage::Refined {
                    let _ = event_tx.send(ShotEvent::Committed {
                        shot_seq,
                        time_to_impact_secs: planned.target.time_secs,
                        duration_secs: trajectory.duration_secs,
                        impact: planned.target.position,
                        rail_start: trajectory.rail.start,
                        rail_end: trajectory.rail.end,
                        peak_joint_speed: trajectory.peak_joint_speed(),
                    });
                    return CommitOutcome::Committed(Box::new(planned.trajectory));
                }
                provisional_sent = true;
                info!(
                    shot = shot_seq,
                    planning_ms = f2(planning_secs * 1e3),
                    detection_to_command_ms = f2(request.age_secs() * 1e3),
                    "real shot: 1차 예측 위치로 이동 시작"
                );
            }
            Err(
                PositionControlError::Stale { .. } | PositionControlError::InsufficientTime { .. },
            ) => {
                debug!(shot = shot_seq, "목표 도착 시각 경과 — 새 궤적 대기");
            }
            Err(
                error
                @ (PositionControlError::Unreachable(_) | PositionControlError::InvalidTarget),
            ) => {
                if request.stage == PredictionStage::Refined {
                    let _ = event_tx.send(ShotEvent::Infeasible {
                        shot_seq,
                        reason: error.to_string(),
                    });
                    return CommitOutcome::Infeasible;
                }
                debug!(shot = shot_seq, %error, "1차 목표 계획 실패 — 새 예측 대기");
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
    verify_target_pose(hardware, &trajectory);
    return Ok(());
}

/// 목표 위치에 대고 난 뒤 샷 시작 자세로 복귀한다.
fn move_to_pose(
    hardware: &mut dyn Hardware,
    arm: &Arm,
    target: &robot::Pose,
) -> Result<(), MoveError> {
    let start = hardware.read_pose().map_err(MoveError::Hardware)?;
    let trajectory = Planner::move_to(arm, &start, target.joints.clone(), target.rail_x)
        .map_err(MoveError::Plan)?;
    hardware.command(&trajectory).map_err(MoveError::Hardware)?;
    wait_idle(hardware);
    verify_target_pose(hardware, &trajectory);
    return Ok(());
}

/// 실제로 명령한 정지 자세에 도착했는지 되읽어 확인한다.
///
/// 여기까지 왔다는 건 "명령을 보냈다"는 뜻이지 "도착했다"는 뜻이 아니다. 실기에서
/// 조용히 안 돌아올 수 있는 경로가 둘 있다: [`RealHardware::command`]는 이미 스윙
/// 실행 중이면 **명령을 버리고 `Ok`를 돌려주고**, `is_busy`는 관절 스트리밍만 보고
/// AXL 레일 이동은 안 본다 — 둘 다 "센터 이동 — 팔이 움직입니다" 로그는 그대로 찍힌다.
/// 로그만 보고 복귀했다고 믿을 수 없으므로 잔차를 숫자로 남긴다.
fn verify_target_pose(hardware: &mut dyn Hardware, trajectory: &motion::Trajectory) {
    let Ok(actual) = hardware.read_pose() else {
        warn!("목표 자세 확인 실패 — 포즈를 못 읽었다");
        return;
    };
    let target = trajectory.end_joints();
    let worst_joint = actual
        .joints
        .values
        .iter()
        .zip(target.values.iter())
        .map(|(actual, target)| (actual - target).abs())
        .fold(0.0_f64, f64::max);
    let rail_error = (actual.rail_x - trajectory.follow_through_rail_x).abs();

    // 임팩트 추종 오차가 0.01 rad(0.6°) 수준이니, 정지 자세에서 그보다 훨씬 큰
    // 잔차는 "명령이 무시됐다"는 신호다.
    const JOINT_TOLERANCE_RAD: f64 = 0.05;
    const RAIL_TOLERANCE_M: f64 = 0.02;
    if worst_joint > JOINT_TOLERANCE_RAD || rail_error > RAIL_TOLERANCE_M {
        warn!(
            worst_joint_rad = f2(worst_joint),
            worst_joint_deg = f2(worst_joint.to_degrees()),
            rail_error_m = f2(rail_error),
            actual = f2_slice(&actual.joints.values),
            target = f2_slice(&target.values),
            "real shot: 목표 자세 미도달 — 명령은 보냈는데 팔이 그 자리에 없다"
        );
    } else {
        info!(
            worst_joint_deg = f2(worst_joint.to_degrees()),
            rail_error_m = f2(rail_error),
            "real shot: 목표 자세 도착 확인"
        );
    }
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
