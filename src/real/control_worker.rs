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

use crossbeam_channel::{Receiver, Sender, select};
use pingpong_bot::defaults;
use pingpong_bot::error::{DomainError, HwError, SwingPlanError};
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::motion::{self, HitPlane, InterceptWindow, Planner};
use pingpong_bot::robot::{self, Arm};
use tracing::{debug, info, info_span, warn};

use super::fmt::{f2, f2_slice};
use super::{
    CommitRequest, ControlStatus, PoseMsg, ShotEvent, Shutdown, SimUpdate, SwingMsg, TrackRequest,
};

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
#[allow(clippy::too_many_arguments)]
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    home: bool,
    coarse_track_enabled: bool,
    // 로봇이 칠 수 있는 평면들. 비전이 준 궤적을 여기서 자른다 — 도달 범위는 로봇 것이라
    // 비전이 알 이유가 없다.
    intercept: InterceptWindow,
    rx: Receiver<CommitRequest>,
    track_rx: Receiver<TrackRequest>,
    status_tx: Sender<ControlStatus>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
    return thread::spawn(move || {
        let _span = info_span!("control").entered();
        let planes = intercept.hit_planes();

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
                &planes,
                &rx,
                &track_rx,
                coarse_track_enabled,
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
#[allow(clippy::too_many_arguments)]
fn wait_for_commit(
    hardware: &mut dyn Hardware,
    arm: &Arm,
    planes: &[HitPlane],
    rx: &Receiver<CommitRequest>,
    track_rx: &Receiver<TrackRequest>,
    coarse_track_enabled: bool,
    sim_tx: Option<&Sender<SimUpdate>>,
    event_tx: &Sender<ShotEvent>,
    shot_seq: u64,
    shutdown: &Shutdown,
) -> CommitOutcome {
    let mut last_attempt: Option<Instant> = None;
    let mut last_warn = Instant::now() - Duration::from_secs(10);
    let mut stale = 0_u64;
    let mut coarse = CoarseTrack::default();

    loop {
        if shutdown.is_down() {
            return CommitOutcome::Disconnected;
        }
        // 커밋과 선추종을 함께 기다린다. 커밋이 오면 그걸 우선 처리하고, 그 전까지는
        // 선추종으로 팔을 임팩트 쪽에 붙여 둔다.
        let request = select! {
            recv(rx) -> received => match received {
                Ok(request) => request,
                Err(_) => return CommitOutcome::Disconnected,
            },
            recv(track_rx) -> received => {
                match received {
                    Ok(track) => {
                        if coarse_track_enabled {
                            coarse_track(hardware, arm, planes, &track, &mut coarse);
                        }
                    }
                    Err(_) => return CommitOutcome::Disconnected,
                }
                continue;
            },
            default(RECV_TIMEOUT) => {
                // 선추종으로 팔을 옮겨 놓고 공 신호가 끊기면 그 자세에 굳어버린다.
                // 스윙을 했든 못 했든 홈으로 돌아가야 다음 급구를 받을 수 있다.
                if coarse.moved_away
                    && coarse
                        .last_signal
                        .is_none_or(|at| at.elapsed().as_secs_f64() >= COARSE_IDLE_HOME_SECS)
                {
                    info!("real shot: 공 신호 끊김 — 선추종 자세에서 홈 복귀");
                    return CommitOutcome::BallGone;
                }
                continue;
            }
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

        match Planner::plan_best(arm, &request.predictions(planes), &start) {
            Ok(planned) => {
                // 선추종 이동이 아직 돌고 있으면 여기서 끊는다. `command`는 busy면
                // **조용히 무시하고 `Ok`를 돌려주므로**, 안 끊으면 스윙이 통째로 사라진다.
                if hardware.is_busy() {
                    hardware.cancel();
                    wait_idle(hardware);
                }
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
                    ball_y = f2(request.ball_y()),
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
                    candidates = request.predictions(planes).len(),
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
    verify_centered(hardware, &trajectory);
    return Ok(());
}

/// 실제로 센터에 도착했는지 되읽어 확인한다.
///
/// 여기까지 왔다는 건 "명령을 보냈다"는 뜻이지 "도착했다"는 뜻이 아니다. 실기에서
/// 조용히 안 돌아올 수 있는 경로가 둘 있다: [`RealHardware::command`]는 이미 스윙
/// 실행 중이면 **명령을 버리고 `Ok`를 돌려주고**, `is_busy`는 관절 스트리밍만 보고
/// AXL 레일 이동은 안 본다 — 둘 다 "센터 이동 — 팔이 움직입니다" 로그는 그대로 찍힌다.
/// 로그만 보고 복귀했다고 믿을 수 없으므로 잔차를 숫자로 남긴다.
fn verify_centered(hardware: &mut dyn Hardware, trajectory: &motion::Trajectory) {
    let Ok(actual) = hardware.read_pose() else {
        warn!("센터 복귀 확인 실패 — 포즈를 못 읽었다");
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
            "real shot: 센터 복귀 미완 — 명령은 보냈는데 팔이 그 자리에 없다"
        );
    } else {
        info!(
            worst_joint_deg = f2(worst_joint.to_degrees()),
            rail_error_m = f2(rail_error),
            "real shot: 센터 복귀 확인"
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

/// coarse 선추종 상태 — 쫓을 평면과 마지막 명령 시각.
#[derive(Default)]
struct CoarseTrack {
    /// [`Planner::best_scored_coarse_plane_y`]가 고른 평면 y. 평면마다 IK가 필요해 비싸므로
    /// [`COARSE_RESCORE_THROTTLE_SECS`]마다만 다시 고르고 그 사이엔 캐시를 쓴다.
    preferred_y: Option<f64>,
    last_rescore: Option<Instant>,
    last_command: Option<Instant>,
    /// 선추종 명령을 한 번이라도 보냈는가 — 홈 복귀가 필요한지 판단한다.
    moved_away: bool,
    /// 마지막으로 선추종 요청을 받은 시각.
    last_signal: Option<Instant>,
}

/// 평면 재채점 주기 — sim `COARSE_TRACK_RESCORE_THROTTLE_SECS`와 같은 값.
const COARSE_RESCORE_THROTTLE_SECS: f64 = 0.020;
/// 선추종 명령 주기. 매 프레임 새 궤적을 던지면 실행 중인 이동을 계속 갈아엎는다.
const COARSE_COMMAND_THROTTLE_SECS: f64 = 0.060;
/// 선추종 궤적이 남은 시간을 다 먹으면 안 된다 — 커밋이 오면 즉시 넘겨줘야 한다.
const COARSE_MAX_LEAD_SECS: f64 = 0.25;
/// 선추종 후 이만큼 아무 신호가 없으면 공이 사라진 것으로 보고 홈으로 돌아간다.
const COARSE_IDLE_HOME_SECS: f64 = 0.50;

/// 커밋 전에 팔을 예측 임팩트 쪽으로 미리 옮긴다.
///
/// sim은 미드코트 대기 중 이걸 매 물리 틱 한다(`world.rs`). real엔 없어서 스윙이 센터에서
/// 출발해 **이동과 타격을 한 궤적에 몰아넣었고**, 그래서 요구 관절속도가 한계의 1.38~1.87배로
/// 나왔다. 같은 플래너(`plan_best`)를 쓰면서 결과가 달랐던 진짜 이유가 여기다.
///
/// sim과 다른 점은 인터페이스뿐이다. sim은 `set_rail_target`+`slew_targets_toward`로 목표만
/// 갈아끼우는데, [`Hardware`]에는 궤적 명령밖에 없어서 짧은 이동 궤적을 스로틀해 보낸다.
fn coarse_track(
    hardware: &mut dyn Hardware,
    arm: &Arm,
    planes: &[HitPlane],
    request: &TrackRequest,
    state: &mut CoarseTrack,
) {
    state.last_signal = Some(Instant::now());
    // 스윙이 실행 중이면 끼어들지 않는다 — `command`는 busy면 조용히 무시하므로
    // 여기서 걸러야 "보냈는데 안 갔다"가 안 생긴다.
    if hardware.is_busy() {
        return;
    }
    if state
        .last_command
        .is_some_and(|at| at.elapsed().as_secs_f64() < COARSE_COMMAND_THROTTLE_SECS)
    {
        return;
    }
    let start = match hardware.read_pose() {
        Ok(pose) => pose,
        Err(error) => {
            warn!(%error, "선추종 포즈 읽기 실패");
            return;
        }
    };

    // 쫓을 평면은 최종 커밋과 **같은 점수**로 고른다. 기하 최근접으로 고르면 비행 내내
    // 엉뚱한 평면을 쫓다가 커밋 순간 크게 보정하게 되고, sim 실측에서 그게 net-clear율을
    // 무너뜨렸다 (`coarse_track_geometry_scored` 참고).
    if state
        .last_rescore
        .is_none_or(|at| at.elapsed().as_secs_f64() >= COARSE_RESCORE_THROTTLE_SECS)
    {
        state.last_rescore = Some(Instant::now());
        state.preferred_y =
            Planner::best_scored_coarse_plane_y(arm, &request.predictions(planes), &start);
    }

    let Some((rail_x, joints)) = Planner::plan_coarse_track_targets_for_plane(
        arm,
        &request.predictions(planes),
        state.preferred_y,
    ) else {
        return;
    };

    // 회전 관절은 **부분만** 옮긴다. coarse 목표는 평면 하나의 자세라 거기 과하게 커밋할수록
    // 나머지 후보 평면까지의 Δq가 커진다 — sim 계측에서 0.80이 통과 평면 수의 천장이었다.
    // 레일은 IK 없이 순수 기하라 IK가 수렴 못 해도 선추종이 죽지 않는다.
    let target_joints = match joints {
        Some(joints) => {
            let mut blended = joints;
            for (index, value) in blended.values.iter_mut().enumerate() {
                let rest = arm
                    .default_joints
                    .values
                    .get(index)
                    .copied()
                    .unwrap_or(*value);
                *value = rest + (*value - rest) * defaults::COARSE_TRACK_JOINT_FRACTION;
            }
            blended
        }
        None => start.joints.clone(),
    };

    let trajectory = match Planner::move_to(arm, &start, target_joints, rail_x) {
        Ok(trajectory) => trajectory,
        Err(error) => {
            debug!(%error, "선추종 계획 실패 — 이번 틱 건너뜀");
            return;
        }
    };
    // 남은 시간을 다 먹는 이동은 보내지 않는다. 커밋이 곧 올 텐데 `command`가 busy를
    // 이유로 그걸 버리면 공을 통째로 놓친다.
    if trajectory.duration_secs > COARSE_MAX_LEAD_SECS {
        return;
    }

    state.last_command = Some(Instant::now());
    state.moved_away = true;
    if let Err(error) = hardware.command(&trajectory) {
        warn!(%error, "선추종 명령 실패");
    }
}
