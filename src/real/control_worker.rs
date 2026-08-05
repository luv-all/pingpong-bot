//! 실물 공 위치·높이 정렬 제어 워커.
//!
//! `run`이 워커 시작 전에 레일과 4축 Dynamixel을 최초 중립 자세에 둔다.
//! 이후 공 하나당 예측 위치에 라켓 중심을 정지 정렬한다. 백스윙·임팩트 속도·
//! 팔로스루는 사용하지 않고, 잠시 유지한 뒤 같은 중립 자세로 복귀한다.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::error::{DomainError, HwError};
use pingpong_bot::hardware::dynamixel::{DynamixelConfig, MotorMapping};
use pingpong_bot::hardware::{AppliedRailRacketCommand, Hardware};
use pingpong_bot::robot::{Arm, Joints};
use pingpong_bot::robot::control::{
    DirectControlCommand, DirectControlMeasurement, DirectController, PredictionStage,
};
use pingpong_bot::robot::motion::{self, Planner};
use tracing::{debug, info, info_span, warn};

use super::fmt::{f2, f4};
use super::{CommitRequest, ControlStateSnapshot, PoseMsg, RuntimeEvent, Shutdown, SimUpdate};

const MAX_REQUEST_AGE_SECS: f64 = 0.050;
const COMMAND_THROTTLE: Duration = Duration::from_millis(20);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const BUSY_POLL: Duration = Duration::from_millis(5);
const VERIFY_POLL_PERIOD: Duration = Duration::from_millis(20);
const VERIFY_STABLE_SAMPLES: u8 = 2;
const MAX_CONSECUTIVE_MISSES: u8 = 3;
const RAIL_ERROR_WARN_M: f64 = 0.020;
const AIM_ERROR_WARN_RAD: f64 = 3.0_f64.to_radians();
const STARTUP_SETTLE_TIMEOUT: Duration = Duration::from_secs(10);
const STARTUP_JOINT_TOLERANCE_RAD: f64 = 3.0_f64.to_radians();
const STARTUP_TRIM_DELAY: Duration = Duration::from_secs(1);
const STARTUP_MAX_TRIM_ATTEMPTS: u8 = 2;
const STARTUP_MAX_TRIM_STEP_RAD: f64 = 5.0_f64.to_radians();
// 작은 정상상태 오차에서 통신 진단/재부팅을 시도하지 않는다. 모터가 실제로
// 멈췄다고 볼 만큼 크게 어긋난 경우에만 자동 복구 대상을 확인한다.
const STARTUP_RECOVERY_MIN_ERROR_RAD: f64 = 10.0_f64.to_radians();
const STARTUP_STABLE_SAMPLES: u8 = 2;

#[derive(Default)]
struct CommandLatch {
    track_seq: Option<u64>,
    provisional_sent: bool,
    refined_sent: bool,
}

impl CommandLatch {
    fn should_send(&mut self, track_seq: u64, stage: PredictionStage) -> bool {
        if self.track_seq != Some(track_seq) {
            *self = Self::default();
            self.track_seq = Some(track_seq);
        }
        return match stage {
            PredictionStage::Provisional => !self.provisional_sent,
            PredictionStage::Refined => !self.refined_sent,
        };
    }

    fn mark_sent(&mut self, stage: PredictionStage) {
        match stage {
            PredictionStage::Provisional => self.provisional_sent = true,
            PredictionStage::Refined => self.refined_sent = true,
        }
    }

    /// 이 공의 처리가 끝났다 — 성공·계획 생략 모두 같은 track의 재시도를 막는다.
    fn mark_finished(&mut self) {
        self.provisional_sent = true;
        self.refined_sent = true;
    }
}

/// 위치 정렬 완료 후 실측 비교용 — 복귀 직전에 로그로 남긴다.
struct PendingAlignmentMeasurement {
    track_seq: u64,
    rail_commanded_m: f64,
    joints_commanded: pingpong_bot::robot::Joints,
}

/// 현재 공 하나의 처리 상태.
///
/// `Aligning`의 세 필드는 항상 함께 만들어지고 함께 사라진다 — 예전에는 별도
/// `Option` 세 개(`track_seq`, `return_due_at`,
/// `pending_impact_measurement`)로 표현해 그 불변식이 관례로만 유지됐다.
enum BallControlState {
    Idle,
    Aligning {
        track_seq: u64,
        return_due_at: Instant,
        measurement: PendingAlignmentMeasurement,
    },
}

impl BallControlState {
    /// 이 상태가 주어진 `track_seq`의 추가 명령을 막는가.
    fn blocks(&self, track_seq: u64) -> bool {
        return matches!(
            self,
            BallControlState::Aligning { track_seq: active, .. } if *active == track_seq
        );
    }
}

/// 명령 후 레일·조준축 재측정 상태.
///
/// **현재 실기 루프에서 도달 불가.** `spawn()`의 while 루프는 `pending_verification`을
/// 초기값 `None`과 명령 직후 재설정 `None` 외에는 `Some(...)`으로 대입하지 않는다.
/// 즉 `verify_due_command`의 수렴 판정·타임아웃·`consecutive_misses` 3회 연속
/// 중단 경로는 이 구조체를 직접 구성해 호출하는 유닛 테스트에서만 실행된다.
/// 부활 또는 제거는 별도 결정 사항으로 남아 있다 —
/// `docs/superpowers/specs/2026-08-05-control-worker-state-machine-design.md` 참고.
/// 이번 패스는 동작을 바꾸지 않는다.
struct PendingVerification {
    track_seq: u64,
    command: DirectControlCommand,
    applied: AppliedRailRacketCommand,
    issued_at: Instant,
    next_check_at: Instant,
    deadline: Instant,
    stable_samples: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VerificationResult {
    Pending,
    Succeeded,
    Missed,
}

/// 제어 워커를 띄운다. 실제 장비 동작은 이 워커를 실기 PC에서 실행할 때만 발생한다.
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    rx: Receiver<CommitRequest>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<RuntimeEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
    return thread::spawn(move || {
        let _span = info_span!("control").entered();

        let pose = match hardware.read_pose() {
            Ok(pose) => pose,
            Err(error) => {
                let _ = event_tx.send(RuntimeEvent::Failed {
                    track_seq: None,
                    reason: format!("시작 포즈 읽기 실패: {error}"),
                });
                let _ = event_tx.send(RuntimeEvent::Done);
                return;
            }
        };
        let window = motion::InterceptWindow::default();
        let controller = DirectController::new(window.y_min, window.y_max)
            .expect("기본 레일·라켓 조준 제어 설정");

        if let Some(sim_tx) = &sim_tx {
            let _ = sim_tx.try_send(SimUpdate {
                pose: Some(PoseMsg::from(&pose)),
                ..SimUpdate::default()
            });
        }
        let _ = event_tx.send(RuntimeEvent::Ready { pose });
        let _ = event_tx.send(RuntimeEvent::ControlState {
            state: ControlStateSnapshot::Idle,
        });
        info!("공 위치·방향 정렬 준비 — 스윙 없이 목표 자세로 이동");

        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
        let mut state = BallControlState::Idle;
        let mut consecutive_misses: u8 = 0;

        while !shutdown.is_down() {
            match verify_due_command(
                hardware.as_mut(),
                &mut pending_verification,
                sim_tx.as_ref(),
            ) {
                VerificationResult::Succeeded => consecutive_misses = 0,
                VerificationResult::Missed => {
                    consecutive_misses = consecutive_misses.saturating_add(1);
                    if consecutive_misses >= MAX_CONSECUTIVE_MISSES {
                        hardware.cancel();
                        let _ = event_tx.send(RuntimeEvent::Failed {
                            track_seq: latch.track_seq,
                            reason: format!(
                                "레일·조준축 수렴 실패 {consecutive_misses}회 연속 — 제어 중단"
                            ),
                        });
                        break;
                    }
                }
                VerificationResult::Pending => {}
            }
            let due_for_return = match &state {
                BallControlState::Aligning { return_due_at, .. } => {
                    Instant::now() >= *return_due_at
                }
                BallControlState::Idle => false,
            };
            if pending_verification.is_none() && due_for_return && !hardware.is_busy() {
                if let BallControlState::Aligning { measurement, .. } = &state {
                    match hardware.read_pose() {
                        Ok(measured) => {
                            let joint_errors: Vec<f64> = measurement
                                .joints_commanded
                                .values
                                .iter()
                                .zip(&measured.joints.values)
                                .map(|(commanded, measured)| commanded - measured)
                                .collect();
                            info!(
                                track_seq = measurement.track_seq,
                                rail_commanded_m = f4(measurement.rail_commanded_m),
                                rail_measured_m = f4(measured.rail_x),
                                rail_commanded_minus_measured_m =
                                    f4(measurement.rail_commanded_m - measured.rail_x),
                                joints_commanded = %format!("{:?}", measurement.joints_commanded.values),
                                joints_measured = %format!("{:?}", measured.joints.values),
                                joints_commanded_minus_measured = %format!("{joint_errors:?}"),
                                "공 위치·높이 정렬 완료 후 실측"
                            );
                        }
                        Err(error) => warn!(%error, "공 위치·높이 정렬 완료 후 포즈 읽기 실패"),
                    }
                }
                if let Err(error) = move_to_center(hardware.as_mut(), &arm) {
                    let reason = format!("제어 후 중앙 복귀 실패 — 현재 자세 유지: {error}");
                    warn!(%error, "안전한 중앙 복귀 궤적 없음 — 명령하지 않고 다음 공을 기다린다");
                    let fatal_hardware_error = matches!(error, MoveError::Hardware(_));
                    let _ = event_tx.send(RuntimeEvent::Failed {
                        track_seq: latch.track_seq,
                        reason,
                    });
                    state = BallControlState::Idle;
                    let _ = event_tx.send(RuntimeEvent::ControlState {
                        state: ControlStateSnapshot::Idle,
                    });
                    if fatal_hardware_error {
                        break;
                    }
                    continue;
                }
                match hardware.read_pose() {
                    Ok(pose) => {
                        if let Some(sim_tx) = &sim_tx {
                            let _ = sim_tx.try_send(SimUpdate {
                                pose: Some(PoseMsg::from(&pose)),
                                ..SimUpdate::default()
                            });
                        }
                        info!(track_seq = latch.track_seq, "제어 후 중앙 복귀 완료");
                    }
                    Err(error) => warn!(%error, "중앙 복귀 후 포즈 읽기 실패"),
                }
                state = BallControlState::Idle;
                let _ = event_tx.send(RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Idle,
                });
            }
            let now = Instant::now();
            let mut timeout = pending_verification
                .as_ref()
                .map_or(RECV_TIMEOUT, |pending| {
                    pending
                        .next_check_at
                        .saturating_duration_since(now)
                        .min(RECV_TIMEOUT)
                });
            if pending_verification.is_none()
                && let BallControlState::Aligning { return_due_at, .. } = &state
            {
                let return_wait = if *return_due_at <= now && hardware.is_busy() {
                    BUSY_POLL
                } else {
                    return_due_at.saturating_duration_since(now)
                };
                timeout = timeout.min(return_wait);
            }
            let request = match rx.recv_timeout(timeout) {
                Ok(request) => request,
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            };
            if !latch.should_send(request.track_seq, request.stage)
                || state.blocks(request.track_seq)
                || request.age_secs() > MAX_REQUEST_AGE_SECS
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
            {
                continue;
            }

            let start = match hardware.read_pose() {
                Ok(pose) => pose,
                Err(error) => {
                    warn!(track_seq = request.track_seq, %error, "명령 직전 포즈 읽기 실패");
                    continue;
                }
            };
            let target = match controller.select_target(&request.trajectory) {
                Ok(target) => target,
                Err(error) => {
                    debug!(track_seq = request.track_seq, %error, "공 위치 정렬 목표 선택 생략");
                    continue;
                }
            };
            if let Some(previous) = pending_verification.take() {
                log_verification(&previous, &start, "superseded", false);
            }
            let issued_at = Instant::now();
            let alignment = match Planner::ball_alignment(&arm, &start, target.position) {
                Ok(alignment) => alignment,
                Err(error) => {
                    latch.mark_finished();
                    let _ = event_tx.send(RuntimeEvent::Failed {
                        track_seq: Some(request.track_seq),
                        reason: format!("이번 공 건너뜀 — 위치·방향 정렬 계획 불가: {error}"),
                    });
                    continue;
                }
            };
            let rail_commanded_m = alignment.rail.end;
            let aim_commanded_rad = alignment
                .end
                .values
                .get(pingpong_bot::robot::control::DIRECT_AIM_JOINT_INDEX)
                .copied()
                .unwrap_or(0.0);
            if let Err(error) = hardware.command(&alignment) {
                let _ = event_tx.send(RuntimeEvent::Failed {
                    track_seq: Some(request.track_seq),
                    reason: format!("위치·방향 정렬 명령 실패: {error}"),
                });
                break;
            }
            latch.mark_sent(request.stage);
            last_command = Some(issued_at);
            let _ = event_tx.send(RuntimeEvent::Commanded {
                track_seq: request.track_seq,
                stage: request.stage,
                target: target.position,
                rail_x: rail_commanded_m,
                aim_rad: aim_commanded_rad,
            });

            let return_due_at = issued_at + Duration::from_secs_f64(alignment.duration_secs);
            state = BallControlState::Aligning {
                track_seq: request.track_seq,
                return_due_at,
                measurement: PendingAlignmentMeasurement {
                    track_seq: request.track_seq,
                    rail_commanded_m,
                    joints_commanded: alignment.follow_through.clone(),
                },
            };
            // 같은 공의 반복 정렬 명령은 막고, 새 track에서 다시 정렬한다.
            latch.mark_finished();
            pending_verification = None;
            let _ = event_tx.send(RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Aligning {
                    track_seq: request.track_seq,
                    return_due_at,
                    rail_commanded_m,
                    aim_commanded_rad,
                },
            });

            info!(
                stage = ?request.stage,
                target_x = f4(target.position.x),
                target_y = f4(target.position.y),
                target_z = f4(target.position.z),
                rail_commanded_m = f4(rail_commanded_m),
                aim_commanded_rad = f4(aim_commanded_rad),
                alignment_duration_secs = f4(alignment.duration_secs),
                joints_commanded = %format!("{:?}", alignment.follow_through.values),
                "레일·팔 동시 위치·방향 정렬 시작 — 스윙 없음"
            );
        }

        let _ = event_tx.send(RuntimeEvent::Done);
    });
}

fn verify_due_command(
    hardware: &mut dyn Hardware,
    pending: &mut Option<PendingVerification>,
    sim_tx: Option<&Sender<SimUpdate>>,
) -> VerificationResult {
    if pending
        .as_ref()
        .is_none_or(|verification| Instant::now() < verification.next_check_at)
    {
        return VerificationResult::Pending;
    }
    let now = Instant::now();
    let pose = match hardware.read_pose() {
        Ok(pose) => pose,
        Err(error) => {
            let verification = pending.as_mut().expect("시간이 된 명령 측정");
            if now < verification.deadline {
                verification.next_check_at = now + VERIFY_POLL_PERIOD;
                debug!(track_seq = verification.track_seq, %error, "명령 후 재측정 재시도");
                return VerificationResult::Pending;
            }
            warn!(
                track_seq = verification.track_seq,
                %error,
                "명령 후 레일·조준축 재측정 타임아웃"
            );
            pending.take();
            return VerificationResult::Missed;
        }
    };
    let verification = pending.as_mut().expect("시간이 된 명령 측정");
    let Some(measurement) = DirectControlMeasurement::from_commanded(
        verification.applied.rail_m,
        verification.applied.aim_rad,
        &pose,
    ) else {
        warn!(
            track_seq = verification.track_seq,
            measured_joint_count = pose.joints.values.len(),
            "명령 후 재측정에 라켓 조준축이 없음"
        );
        pending.take();
        return VerificationResult::Missed;
    };
    if let Some(sim_tx) = sim_tx {
        let _ = sim_tx.try_send(SimUpdate {
            pose: Some(PoseMsg::from(&pose)),
            ..SimUpdate::default()
        });
    }

    let within_tolerance = measurement.rail_error_m.abs() <= RAIL_ERROR_WARN_M
        && measurement.aim_error_rad.abs() <= AIM_ERROR_WARN_RAD;
    if within_tolerance {
        verification.stable_samples = verification.stable_samples.saturating_add(1);
    } else {
        verification.stable_samples = 0;
    }
    if verification.stable_samples >= VERIFY_STABLE_SAMPLES {
        let verification = pending.take().expect("수렴한 명령");
        log_verification(&verification, &pose, "converged", false);
        return VerificationResult::Succeeded;
    }
    if now >= verification.deadline {
        let verification = pending.take().expect("타임아웃 명령");
        log_verification(&verification, &pose, "timeout", true);
        return VerificationResult::Missed;
    }
    verification.next_check_at = now + VERIFY_POLL_PERIOD;
    debug!(
        track_seq = verification.track_seq,
        stage = ?verification.command.stage,
        rail_error_m = f4(measurement.rail_error_m),
        aim_error_deg = f2(measurement.aim_error_rad.to_degrees()),
        "레일·조준축 수렴 대기"
    );
    return VerificationResult::Pending;
}

fn log_verification(
    verification: &PendingVerification,
    pose: &pingpong_bot::robot::Pose,
    outcome: &'static str,
    warning: bool,
) {
    let Some(measurement) = DirectControlMeasurement::from_commanded(
        verification.applied.rail_m,
        verification.applied.aim_rad,
        pose,
    ) else {
        return;
    };
    let outside_tolerance = measurement.rail_error_m.abs() > RAIL_ERROR_WARN_M
        || measurement.aim_error_rad.abs() > AIM_ERROR_WARN_RAD;
    let log_measurement = || {
        (
            measurement.rail_commanded_m,
            measurement.rail_measured_m,
            measurement.rail_error_m,
            measurement.aim_commanded_rad.to_degrees(),
            measurement.aim_measured_rad.to_degrees(),
            measurement.aim_error_rad.to_degrees(),
        )
    };
    let (
        rail_commanded_m,
        rail_measured_m,
        rail_error_m,
        aim_commanded_deg,
        aim_measured_deg,
        aim_error_deg,
    ) = log_measurement();
    if warning || outside_tolerance {
        warn!(
            track_seq = verification.track_seq,
            stage = ?verification.command.stage,
            outcome,
            elapsed_ms = f2(verification.issued_at.elapsed().as_secs_f64() * 1e3),
            target_x = f4(verification.command.target.position.x),
            target_y = f4(verification.command.target.position.y),
            target_z = f4(verification.command.target.position.z),
            rail_requested_m = f4(verification.command.rail_x),
            rail_commanded_m = f4(rail_commanded_m),
            rail_measured_m = f4(rail_measured_m),
            rail_commanded_minus_measured_m = f4(rail_error_m),
            aim_commanded_rad = f4(measurement.aim_commanded_rad),
            aim_measured_rad = f4(measurement.aim_measured_rad),
            aim_commanded_minus_measured_rad = f4(measurement.aim_error_rad),
            aim_commanded_deg = f2(aim_commanded_deg),
            aim_measured_deg = f2(aim_measured_deg),
            aim_commanded_minus_measured_deg = f2(aim_error_deg),
            "명령 후 제어 수렴 실패"
        );
    } else {
        info!(
            track_seq = verification.track_seq,
            stage = ?verification.command.stage,
            outcome,
            elapsed_ms = f2(verification.issued_at.elapsed().as_secs_f64() * 1e3),
            target_x = f4(verification.command.target.position.x),
            target_y = f4(verification.command.target.position.y),
            target_z = f4(verification.command.target.position.z),
            rail_requested_m = f4(verification.command.rail_x),
            rail_commanded_m = f4(rail_commanded_m),
            rail_measured_m = f4(rail_measured_m),
            rail_commanded_minus_measured_m = f4(rail_error_m),
            aim_commanded_rad = f4(measurement.aim_commanded_rad),
            aim_measured_rad = f4(measurement.aim_measured_rad),
            aim_commanded_minus_measured_rad = f4(measurement.aim_error_rad),
            aim_commanded_deg = f2(aim_commanded_deg),
            aim_measured_deg = f2(aim_measured_deg),
            aim_commanded_minus_measured_deg = f2(aim_error_deg),
            "명령 후 제어 괴리 측정"
        );
    }
}

/// 런타임의 다른 스레드를 띄우기 전에 레일·전체 관절을 준비 자세로 초기화한다.
pub(super) fn initialize_pose(
    hardware: &mut dyn Hardware,
    arm: &Arm,
) -> Result<pingpong_bot::robot::Pose, MoveError> {
    return initialize_pose_attempt(hardware, arm, true);
}

fn initialize_pose_attempt(
    hardware: &mut dyn Hardware,
    arm: &Arm,
    allow_motor_recovery: bool,
) -> Result<pingpong_bot::robot::Pose, MoveError> {
    let measured = hardware.read_pose().map_err(MoveError::Hardware)?;
    if let Some(rail) = arm.rail {
        info!(
            rail_measured_m = f4(measured.rail_x),
            rail_target_m = f4(rail.default_x()),
            rail_commanded_minus_measured_m = f4(rail.default_x() - measured.rail_x),
            configured_min_m = f4(rail.x_min),
            configured_max_m = f4(rail.x_max),
            joints_measured = %format!("{:?}", measured.joints.values),
            "시작 자세 초기화 전 실측"
        );
    }
    // 전원이 꺼진 동안 손으로 팔을 움직였을 수 있다. 현재 자세에서 중립 자세로
    // 곧장 가는 경로가 테이블을 스치면 상승 중간 자세를 거치는 안전 복귀를 쓴다.
    let trajectories = plan_neutral_return_segments(arm, &measured).map_err(MoveError::Plan)?;
    let ready_joints = arm.default_joints.clone();
    let ready_rail_x = arm
        .rail
        .as_ref()
        .map_or(measured.rail_x, |rail| rail.default_x());
    let mapping = MotorMapping::new(DynamixelConfig::default()).map_err(|error| {
        MoveError::Hardware(HwError::InvalidConfig {
            reason: error.to_string(),
        })
    })?;
    let goal_ticks: Vec<i32> = ready_joints
        .values
        .iter()
        .enumerate()
        .map(|(index, angle)| mapping.radians_to_ticks(index, *angle))
        .collect();
    let goal_joint_deg: Vec<f64> = ready_joints
        .values
        .iter()
        .map(|angle| angle.to_degrees())
        .collect();
    info!(
        rail_target_m = f4(ready_rail_x),
        joints_target_rad = %format!("{:?}", ready_joints.values),
        joints_target_deg = %format!("{goal_joint_deg:?}"),
        dynamixel_goal_ticks = %format!("{goal_ticks:?}"),
        joint_signs = %format!("{:?}", mapping.config().joint_signs),
        joint_offsets_rad = %format!("{:?}", mapping.config().joint_offsets_rad),
        "시작 자세 공통 목표 — sim·real 동일 논리 좌표"
    );
    if trajectories.len() > 1 {
        info!(
            segments = trajectories.len(),
            "손으로 바뀐 시작 자세 복구 — 상승 중간 자세를 거쳐 중립 자세로 이동"
        );
    }
    for trajectory in trajectories {
        hardware.command(&trajectory).map_err(MoveError::Hardware)?;
        while hardware.is_busy() {
            thread::sleep(BUSY_POLL);
        }
    }
    // executor 종료는 마지막 Goal Position을 보냈다는 뜻일 뿐, 모터가 실제로
    // 도착했다는 뜻은 아니다. 실측이 준비 자세에 연속 두 번 들어올 때까지 기다린다.
    let settle_started = Instant::now();
    let settle_deadline = settle_started + STARTUP_SETTLE_TIMEOUT;
    let mut next_trim_at = settle_started + STARTUP_TRIM_DELAY;
    let mut trim_attempts = 0_u8;
    let mut stable_samples = 0_u8;
    let after = loop {
        let pose = hardware.read_pose().map_err(MoveError::Hardware)?;
        let joint_errors: Vec<f64> = ready_joints
            .values
            .iter()
            .zip(&pose.joints.values)
            .map(|(commanded, measured)| commanded - measured)
            .collect();
        let max_joint_error_rad = joint_errors
            .iter()
            .map(|error| error.abs())
            .fold(0.0_f64, f64::max);
        if max_joint_error_rad <= STARTUP_JOINT_TOLERANCE_RAD {
            stable_samples = stable_samples.saturating_add(1);
            if stable_samples >= STARTUP_STABLE_SAMPLES {
                break pose;
            }
        } else {
            stable_samples = 0;
        }
        // 위치모드인데도 중력·유격으로 3° 부근에서 멎는 실기를 위한 폐루프 미세
        // 보정이다. 목표를 달성하지 못한 축만 남은 오차만큼 더 보내고 다시 실측한다.
        // 고정 3.24° 오프셋이 아니라 매 실행의 실제 오차를 써서 과보정을 피한다.
        if Instant::now() >= next_trim_at
            && trim_attempts < STARTUP_MAX_TRIM_ATTEMPTS
            && max_joint_error_rad < STARTUP_RECOVERY_MIN_ERROR_RAD
        {
            let compensated_values: Vec<f64> = ready_joints
                .values
                .iter()
                .zip(&joint_errors)
                .enumerate()
                .map(|(index, (target, error))| {
                    let correction = if error.abs() > STARTUP_JOINT_TOLERANCE_RAD {
                        error.clamp(-STARTUP_MAX_TRIM_STEP_RAD, STARTUP_MAX_TRIM_STEP_RAD)
                    } else {
                        0.0
                    };
                    let compensated = target + correction;
                    return arm
                        .joint_limit(index)
                        .map_or(compensated, |limit| compensated.clamp(limit.min, limit.max));
                })
                .collect();
            let compensated = Joints::from_slice(&compensated_values);
            let correction_deg: Vec<f64> = compensated
                .values
                .iter()
                .zip(&ready_joints.values)
                .map(|(commanded, target)| (commanded - target).to_degrees())
                .collect();
            let correction = Planner::move_to(arm, &pose, compensated, pose.rail_x)
                .map_err(MoveError::Plan)?;
            trim_attempts += 1;
            info!(
                attempt = trim_attempts,
                correction_deg = %format!("{correction_deg:?}"),
                measured_error_deg = %format!("{:?}", joint_errors.iter().map(|error| error.to_degrees()).collect::<Vec<_>>()),
                "시작 팔 자세 잔여 오차 폐루프 보정"
            );
            hardware
                .command_joints(&correction)
                .map_err(MoveError::Hardware)?;
            while hardware.is_busy() {
                thread::sleep(BUSY_POLL);
            }
            next_trim_at = Instant::now() + STARTUP_TRIM_DELAY;
            continue;
        }
        if Instant::now() >= settle_deadline {
            hardware.log_joint_diagnostics();
            warn!(
                rail_commanded_m = f4(ready_rail_x),
                rail_measured_m = f4(pose.rail_x),
                rail_commanded_minus_measured_m = f4(ready_rail_x - pose.rail_x),
                joints_commanded = %format!("{:?}", ready_joints.values),
                joints_measured = %format!("{:?}", pose.joints.values),
                joints_commanded_minus_measured = %format!("{joint_errors:?}"),
                "시작 팔 자세 실측 수렴 실패 — 관절 부호·영점 보정 필요"
            );
            if allow_motor_recovery && max_joint_error_rad >= STARTUP_RECOVERY_MIN_ERROR_RAD {
                match hardware.recover_joint_control() {
                    Ok(true) => {
                        info!("Dynamixel 자동 복구 완료 — 시작 팔 자세를 한 번 다시 명령");
                        thread::sleep(Duration::from_millis(300));
                        return initialize_pose_attempt(hardware, arm, false);
                    }
                    Ok(false) => {}
                    Err(error) => {
                        warn!(%error, "Dynamixel 자동 복구 실패");
                    }
                }
            }
            return Err(MoveError::StartupAlignmentTimeout {
                max_joint_error_rad,
            });
        }
        thread::sleep(VERIFY_POLL_PERIOD);
    };
    if let Some(rail) = arm.rail {
        let joint_errors: Vec<f64> = ready_joints
            .values
            .iter()
            .zip(&after.joints.values)
            .map(|(commanded, measured)| commanded - measured)
            .collect();
        info!(
            rail_commanded_m = f4(rail.default_x()),
            rail_measured_m = f4(after.rail_x),
            rail_commanded_minus_measured_m = f4(rail.default_x() - after.rail_x),
            joints_commanded = %format!("{:?}", ready_joints.values),
            joints_measured = %format!("{:?}", after.joints.values),
            joints_commanded_minus_measured = %format!("{joint_errors:?}"),
            "시작 자세 초기화 후 실측"
        );
    }
    if let (Some(target_racket), Some(measured_racket)) = (
        arm.forward_kinematics_with_rail(ready_rail_x, &ready_joints),
        arm.forward_kinematics_with_rail(after.rail_x, &after.joints),
    ) {
        let position_error = target_racket.position.coords - measured_racket.position.coords;
        info!(
            racket_target_xyz_m = %format!("{:?}", target_racket.position.coords),
            racket_measured_xyz_m = %format!("{:?}", measured_racket.position.coords),
            racket_target_minus_measured_xyz_m = %format!("{position_error:?}"),
            racket_target_normal = %format!("{:?}", target_racket.normal),
            racket_measured_normal = %format!("{:?}", measured_racket.normal),
            racket_normal_dot = f4(target_racket.normal.dot(&measured_racket.normal)),
            "시작 자세 라켓 FK 얼라인 진단"
        );
    }
    return Ok(after);
}

/// 시작 자세 초기화와 공 제어 후 복귀에 같은 전체축 이동을 사용한다.
fn move_to_center(hardware: &mut dyn Hardware, arm: &Arm) -> Result<(), MoveError> {
    let start = hardware.read_pose().map_err(MoveError::Hardware)?;
    let trajectories = plan_neutral_return_segments(arm, &start).map_err(MoveError::Plan)?;
    if trajectories.len() > 1 {
        info!(
            segments = trajectories.len(),
            "직접 복귀 관통 회피 — 위로 든 뒤 준비 자세 복귀"
        );
    }
    for trajectory in trajectories {
        hardware.command(&trajectory).map_err(MoveError::Hardware)?;
        while hardware.is_busy() {
            thread::sleep(BUSY_POLL);
        }
    }
    return Ok(());
}

/// 직접 복귀가 테이블을 스치면 안전한 상승 중간 자세를 거치는 2구간을 찾는다.
/// 모든 구간은 실행 전에 속도·토크·테이블 충돌 검사를 통과해야 한다.
fn plan_neutral_return_segments(
    arm: &Arm,
    start: &pingpong_bot::robot::Pose,
) -> Result<Vec<pingpong_bot::robot::motion::Trajectory>, DomainError> {
    match Planner::return_to_center(arm, start) {
        Ok(direct) => return Ok(vec![direct]),
        Err(error) => {
            if !matches!(
                error,
                DomainError::InfeasibleSwing(
                    pingpong_bot::error::SwingPlanError::TablePenetration { .. }
                )
            ) {
                return Err(error);
            }
        }
    }

    let racket = arm
        .forward_kinematics_with_rail(start.rail_x, &start.joints)
        .ok_or_else(|| {
            DomainError::InfeasibleSwing(
                pingpong_bot::error::SwingPlanError::InverseKinematicsNoSolution {
                    target_x: start.rail_x,
                    target_y: 0.0,
                    target_z: 0.0,
                },
            )
        })?;
    let mut last_error = None;
    for lift_m in [0.03, 0.06, 0.10, 0.15] {
        let lifted_target = pingpong_bot::Point3::new(
            racket.position.x,
            racket.position.y,
            racket.position.z + lift_m,
        );
        let lifted_joints = match arm.rail.as_ref() {
            Some(rail) => arm.inverse_kinematics_with_rail(
                rail,
                start.rail_x,
                lifted_target,
                Some(&start.joints),
            ),
            None => arm.inverse_kinematics_near(lifted_target, Some(&start.joints)),
        };
        let lifted_joints = match lifted_joints {
            Ok(joints) => joints,
            Err(error) => {
                last_error = Some(DomainError::InfeasibleSwing(error));
                continue;
            }
        };
        let lift = match Planner::move_to(arm, start, lifted_joints, start.rail_x) {
            Ok(trajectory) => trajectory,
            Err(error) => {
                last_error = Some(error);
                continue;
            }
        };
        let lifted_pose =
            pingpong_bot::robot::Pose::new(lift.follow_through_rail_x, lift.follow_through.clone());
        match Planner::return_to_center(arm, &lifted_pose) {
            Ok(ready) => return Ok(vec![lift, ready]),
            Err(error) => last_error = Some(error),
        }
    }
    return Err(last_error.unwrap_or_else(|| {
        DomainError::InfeasibleSwing(
            pingpong_bot::error::SwingPlanError::InverseKinematicsNoSolution {
                target_x: start.rail_x,
                target_y: racket.position.y,
                target_z: racket.position.z,
            },
        )
    }));
}

#[derive(Debug)]
pub(super) enum MoveError {
    Hardware(HwError),
    Plan(DomainError),
    StartupAlignmentTimeout { max_joint_error_rad: f64 },
}

impl std::fmt::Display for MoveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::Hardware(error) => write!(f, "{error}"),
            Self::Plan(error) => write!(f, "{error}"),
            Self::StartupAlignmentTimeout {
                max_joint_error_rad,
            } => write!(
                f,
                "시작 팔 자세가 10초 안에 수렴하지 않음: 최대 관절 오차 {:+.2}°",
                max_joint_error_rad.to_degrees()
            ),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::Point3;
    use pingpong_bot::robot::control::HitTarget;
    use pingpong_bot::robot::{Joints, Pose};

    struct ReadCountingHardware {
        reads: usize,
        pose: Pose,
    }

    impl Hardware for ReadCountingHardware {
        fn command(
            &mut self,
            _trajectory: &pingpong_bot::robot::motion::Trajectory,
        ) -> Result<(), HwError> {
            return Ok(());
        }

        fn read_pose(&mut self) -> Result<Pose, HwError> {
            self.reads += 1;
            return Ok(self.pose.clone());
        }
    }

    struct PoseApplyingHardware {
        pose: Pose,
    }

    impl Hardware for PoseApplyingHardware {
        fn command(
            &mut self,
            trajectory: &pingpong_bot::robot::motion::Trajectory,
        ) -> Result<(), HwError> {
            self.pose = Pose::new(
                trajectory.follow_through_rail_x,
                trajectory.end_joints().clone(),
            );
            return Ok(());
        }

        fn read_pose(&mut self) -> Result<Pose, HwError> {
            return Ok(self.pose.clone());
        }
    }

    #[test]
    fn startup_initialization_sets_ready_rail_and_all_joints() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.x_min, Joints::from_slice(&[0.0; 4])),
        };

        let initialized = initialize_pose(&mut hardware, &robot.arm).expect("initialize");
        let start = Pose::new(rail.x_min, Joints::from_slice(&[0.0; 4]));
        let expected = Planner::return_to_center(&robot.arm, &start).expect("neutral ready");

        assert!((initialized.rail_x - rail.default_x()).abs() < 1e-12);
        assert_eq!(initialized.joints, expected.follow_through);
    }

    #[test]
    fn logged_follow_through_pose_has_a_safe_ready_return() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        // 2026-08-05 실기 로그에서 직접 복귀가 테이블을 2mm 관통했던 실측 자세.
        let start = Pose::new(
            1.258_578,
            Joints::from_slice(&[1.264_000_169, -0.423_378_697, 0.115_048_559, -0.550_699_103]),
        );

        let segments = plan_neutral_return_segments(&robot.arm, &start)
            .expect("직접 또는 상승 중간 자세를 거쳐 안전하게 복귀");
        assert!(!segments.is_empty());
        assert!(segments.len() <= 2);
    }

    #[test]
    fn each_prediction_stage_is_sent_only_once_per_ball() {
        let mut latch = CommandLatch::default();
        assert!(latch.should_send(1, PredictionStage::Provisional));
        latch.mark_sent(PredictionStage::Provisional);
        assert!(!latch.should_send(1, PredictionStage::Provisional));
        assert!(latch.should_send(1, PredictionStage::Refined));
        latch.mark_sent(PredictionStage::Refined);
        assert!(!latch.should_send(1, PredictionStage::Refined));

        assert!(latch.should_send(2, PredictionStage::Provisional));
    }

    #[test]
    fn new_track_resets_latch_before_refined_stage() {
        let mut latch = CommandLatch::default();
        assert!(latch.should_send(1, PredictionStage::Provisional));
        latch.mark_sent(PredictionStage::Provisional);
        assert!(latch.should_send(2, PredictionStage::Provisional));
    }

    #[test]
    fn aligned_track_is_permanently_blocked_even_after_returning_to_idle() {
        let mut latch = CommandLatch::default();
        assert!(latch.should_send(3, PredictionStage::Provisional));
        latch.mark_sent(PredictionStage::Provisional);
        latch.mark_finished();
        assert!(!latch.should_send(3, PredictionStage::Provisional));
        assert!(!latch.should_send(3, PredictionStage::Refined));

        assert!(latch.should_send(4, PredictionStage::Provisional));
    }

    #[test]
    fn due_command_needs_two_stable_readbacks() {
        let command = DirectControlCommand {
            stage: PredictionStage::Refined,
            target: HitTarget {
                position: Point3::new(0.30, 0.20, 0.25),
                incoming_velocity: nalgebra::Vector3::zeros(),
                time_secs: 0.2,
            },
            rail_x: 0.30,
            aim_rad: -0.40,
            duration_secs: 0.1,
        };
        let mut hardware = ReadCountingHardware {
            reads: 0,
            pose: Pose::new(0.29, Joints::from_slice(&[0.0, -0.41, 0.0, 0.0])),
        };
        let mut pending = Some(PendingVerification {
            track_seq: 7,
            command,
            applied: AppliedRailRacketCommand {
                rail_m: 0.30,
                aim_rad: -0.40,
                rail_sent: true,
            },
            issued_at: Instant::now() - Duration::from_millis(100),
            next_check_at: Instant::now() - Duration::from_millis(1),
            deadline: Instant::now() + Duration::from_millis(100),
            stable_samples: 0,
        });

        assert_eq!(
            verify_due_command(&mut hardware, &mut pending, None),
            VerificationResult::Pending
        );
        pending.as_mut().unwrap().next_check_at = Instant::now() - Duration::from_millis(1);
        assert_eq!(
            verify_due_command(&mut hardware, &mut pending, None),
            VerificationResult::Succeeded
        );

        assert_eq!(hardware.reads, 2);
        assert!(pending.is_none());
    }

    #[test]
    fn idle_blocks_nothing() {
        let state = BallControlState::Idle;
        assert!(!state.blocks(1));
        assert!(!state.blocks(999));
    }

    #[test]
    fn aligning_blocks_only_its_own_track() {
        let state = BallControlState::Aligning {
            track_seq: 5,
            return_due_at: Instant::now(),
            measurement: PendingAlignmentMeasurement {
                track_seq: 5,
                rail_commanded_m: 0.30,
                joints_commanded: Joints::from_slice(&[0.0; 4]),
            },
        };
        assert!(state.blocks(5));
        assert!(!state.blocks(6));
    }
}
