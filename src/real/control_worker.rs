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
use pingpong_bot::robot::control::{DirectControlCommand, DirectControlMeasurement};
use pingpong_bot::robot::motion::{self, Planner};
use pingpong_bot::robot::{Arm, Joints};
use pingpong_bot::vision::State as VisionState;
use tracing::{debug, info, info_span, warn};

use super::fmt::{f2, f4};
use super::{
    CommitRequest, ControlStateSnapshot, PoseMsg, RuntimeEvent, Shutdown, SimUpdate, TestControl,
    TestZone,
};

const COMMAND_THROTTLE: Duration = Duration::from_millis(20);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const BUSY_POLL: Duration = Duration::from_millis(5);
const VERIFY_POLL_PERIOD: Duration = Duration::from_millis(20);
const VERIFY_STABLE_SAMPLES: u8 = 2;
const MAX_CONSECUTIVE_MISSES: u8 = 3;
const RAIL_ERROR_WARN_M: f64 = 0.020;
const AIM_ERROR_WARN_RAD: f64 = 3.0_f64.to_radians();
const STARTUP_SETTLE_TIMEOUT: Duration = Duration::from_secs(10);
// 3° 허용치는 모터가 아직 이동 중인 2.5° 오차를 0.35초 만에
// "수렴"으로 판정했다. 반대로 1°는 하드웨어 오류 없이 1.84°에서 안정된
// ID 4를 10초 타임아웃으로 막았다. 실기 추종 편차를 반영해 2°를 쓰되,
// 아래 연속 샘플 조건으로 이동 중 조기 통과를 막는다.
const STARTUP_JOINT_TOLERANCE_RAD: f64 = 2.0_f64.to_radians();
const STARTUP_TRIM_DELAY: Duration = Duration::from_secs(1);
const STARTUP_MAX_TRIM_ATTEMPTS: u8 = 2;
const STARTUP_MAX_TRIM_STEP_RAD: f64 = 5.0_f64.to_radians();
// 작은 정상상태 오차에서 통신 진단/재부팅을 시도하지 않는다. 모터가 실제로
// 멈췄다고 볼 만큼 크게 어긋난 경우에만 자동 복구 대상을 확인한다.
const STARTUP_RECOVERY_MIN_ERROR_RAD: f64 = 10.0_f64.to_radians();
// 20 ms 간격 5회(80 ms 이상) 연속 수렴해야 도착으로 본다.
const STARTUP_STABLE_SAMPLES: u8 = 5;
// 2026-08-05 자·육안 실측. 센서값이 아니라 시작 FK 모델과 비교할 벤치 기준이다.
const BENCH_WRIST_ABOVE_TABLE_M: f64 = 0.340;
const BENCH_RACKET_LOWEST_ABOVE_TABLE_M: f64 = 0.155;
const BENCH_HANDLE_END_ABOVE_TABLE_M: f64 = 0.410;
const BENCH_RACKET_AXIS_FROM_VERTICAL_DEG: f64 = 8.0;
const BENCH_RACKET_TOTAL_LENGTH_M: f64 = 0.255;

#[derive(Default)]
struct CommandLatch {
    track_seq: Option<u64>,
    finished: bool,
}

impl CommandLatch {
    fn should_send(&mut self, track_seq: u64) -> bool {
        if self.track_seq != Some(track_seq) {
            *self = Self::default();
            self.track_seq = Some(track_seq);
        }
        return !self.finished;
    }

    /// 이 공의 처리가 끝났다 — 성공·계획 생략 모두 같은 track의 재시도를 막는다.
    fn mark_finished(&mut self) {
        self.finished = true;
    }
}

/// 새 비전의 전체 예측 궤적에서 제어가 사용할 접수 평면을 고른다.
///
/// 요청이 큐에서 기다린 시간만큼 `at_time(last_measured + age)`로 공을 전진시킨 뒤,
/// 아직 미래인 평면만 남긴다. 비전은 접수 범위를 모르고 제어만 이 정책을 가진다.
fn select_alignment_target(
    request: &CommitRequest,
    window: motion::InterceptWindow,
) -> Result<VisionState, &'static str> {
    let measured_t = request
        .trajectory
        .measured
        .last()
        .map(|state| state.t)
        .ok_or("관측 궤적이 비어 있음")?;
    let age = Duration::try_from_secs_f64(request.age_secs())
        .map_err(|_| "요청 지연 시간이 유효하지 않음")?;
    let effective_now = measured_t.saturating_add(age);
    // 낡았다는 이유로 요청 전체를 버리지 않고 현재 상태를 전진시킨다.
    request
        .trajectory
        .predicted
        .at_time(effective_now)
        .ok_or("요청 지연 뒤 예측 궤적이 이미 끝남")?;

    let center_y = 0.5 * (window.y_min + window.y_max);
    return window
        .hit_planes()
        .into_iter()
        .filter_map(|plane| request.trajectory.predicted.at_plane(plane.y))
        .filter(|state| state.t > effective_now)
        .min_by(|left, right| {
            let left_center = (left.position.y - center_y).abs();
            let right_center = (right.position.y - center_y).abs();
            left_center.total_cmp(&right_center).then_with(|| {
                left.sigma_position
                    .max()
                    .total_cmp(&right.sigma_position.max())
            })
        })
        .ok_or("접수 구간에 아직 도달 가능한 미래 예측이 없음");
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
    test_control_rx: Receiver<TestControl>,
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
        let mut home_rail_x = arm.rail.map(|rail| rail.default_x()).unwrap_or(pose.rail_x);
        let mut current_zone = TestZone::Center;

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
        let _ = event_tx.send(RuntimeEvent::TestZoneChanged {
            zone: current_zone,
            home_rail_x,
        });
        info!("공 위치·방향 정렬 준비 — 스윙 없이 목표 자세로 이동");

        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
        let mut state = BallControlState::Idle;
        let mut consecutive_misses: u8 = 0;
        let mut pending_test_control: Option<TestControl> = None;

        'control: while !shutdown.is_down() {
            while let Ok(control) = test_control_rx.try_recv() {
                match control {
                    TestControl::ResetPosition => {
                        pending_test_control = None;
                        if hardware.is_busy() {
                            hardware.cancel();
                            while hardware.is_busy() && !shutdown.is_down() {
                                thread::sleep(BUSY_POLL);
                            }
                        }
                        if shutdown.is_down() {
                            break 'control;
                        }
                        pending_verification = None;
                        consecutive_misses = 0;
                        match apply_test_control(
                            TestControl::ResetPosition,
                            hardware.as_mut(),
                            &arm,
                            &mut home_rail_x,
                            &mut current_zone,
                            &mut latch,
                            &mut state,
                            sim_tx.as_ref(),
                            &event_tx,
                        ) {
                            Ok(()) => {}
                            Err(MoveError::Hardware(error)) => {
                                let _ = event_tx.send(RuntimeEvent::Failed {
                                    track_seq: latch.track_seq,
                                    reason: format!("수동 리셋 중 하드웨어 오류: {error}"),
                                });
                                break 'control;
                            }
                            Err(error @ MoveError::Plan(_))
                            | Err(error @ MoveError::StartupAlignmentTimeout { .. }) => {
                                warn!(%error, "수동 리셋 중 준비 자세 계획 실패 — 세션은 유지");
                                let _ = event_tx.send(RuntimeEvent::Failed {
                                    track_seq: latch.track_seq,
                                    reason: format!("수동 리셋 중 준비 자세 계획 실패: {error}"),
                                });
                                state = BallControlState::Idle;
                                let _ = event_tx.send(RuntimeEvent::ControlState {
                                    state: ControlStateSnapshot::Idle,
                                });
                            }
                        }
                    }
                    other => pending_test_control = Some(other),
                }
            }
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
            let idle_ready = pending_verification.is_none() && !hardware.is_busy();
            if idle_ready && let Some(control) = pending_test_control.take() {
                consecutive_misses = 0;
                match apply_test_control(
                    control,
                    hardware.as_mut(),
                    &arm,
                    &mut home_rail_x,
                    &mut current_zone,
                    &mut latch,
                    &mut state,
                    sim_tx.as_ref(),
                    &event_tx,
                ) {
                    Ok(()) => {}
                    Err(MoveError::Hardware(error)) => {
                        let _ = event_tx.send(RuntimeEvent::Failed {
                            track_seq: latch.track_seq,
                            reason: format!("테스트 컨트롤 적용 중 하드웨어 오류: {error}"),
                        });
                        break;
                    }
                    Err(error @ MoveError::Plan(_))
                    | Err(error @ MoveError::StartupAlignmentTimeout { .. }) => {
                        warn!(%error, "테스트 컨트롤 적용 중 준비 자세 계획 실패 — 세션은 유지");
                        let _ = event_tx.send(RuntimeEvent::Failed {
                            track_seq: latch.track_seq,
                            reason: format!("테스트 컨트롤 적용 중 준비 자세 계획 실패: {error}"),
                        });
                        state = BallControlState::Idle;
                        let _ = event_tx.send(RuntimeEvent::ControlState {
                            state: ControlStateSnapshot::Idle,
                        });
                    }
                }
            } else if idle_ready && due_for_return {
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
                if let Err(error) = move_to_ready(hardware.as_mut(), &arm, home_rail_x) {
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
            let track_seq = request.track_seq();
            if !latch.should_send(track_seq)
                || state.blocks(track_seq)
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
            {
                continue;
            }

            let start = match hardware.read_pose() {
                Ok(pose) => pose,
                Err(error) => {
                    warn!(track_seq, %error, "명령 직전 포즈 읽기 실패");
                    continue;
                }
            };
            let target = match select_alignment_target(&request, window) {
                Ok(target) => target,
                Err(error) => {
                    debug!(
                        track_seq,
                        reason = error,
                        "새 비전 궤적에서 정렬 목표 선택 생략"
                    );
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
                        track_seq: Some(track_seq),
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
                    track_seq: Some(track_seq),
                    reason: format!("위치·방향 정렬 명령 실패: {error}"),
                });
                break;
            }
            latch.mark_finished();
            last_command = Some(issued_at);
            let _ = event_tx.send(RuntimeEvent::Commanded {
                track_seq,
                target: target.position,
                rail_x: rail_commanded_m,
                aim_rad: aim_commanded_rad,
            });
            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    target: Some(target.position),
                    ..SimUpdate::default()
                });
            }

            let predicted_arrival_at = request.trajectory.origin + target.t;
            let return_due_at = predicted_arrival_at
                + Duration::from_secs_f64(pingpong_bot::defaults::POST_ALIGNMENT_HOLD_SECS);
            state = BallControlState::Aligning {
                track_seq,
                return_due_at,
                measurement: PendingAlignmentMeasurement {
                    track_seq,
                    rail_commanded_m,
                    joints_commanded: alignment.follow_through.clone(),
                },
            };
            // 같은 공의 반복 정렬 명령은 막고, 새 track에서 다시 정렬한다.
            pending_verification = None;
            let _ = event_tx.send(RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Aligning {
                    track_seq,
                    return_due_at,
                    rail_commanded_m,
                    aim_commanded_rad,
                },
            });

            info!(
                track_seq,
                request_age_secs = f4(request.age_secs()),
                target_time_secs = f4(target.t.as_secs_f64()),
                predicted_arrival_in_secs = f4(
                    predicted_arrival_at
                        .saturating_duration_since(Instant::now())
                        .as_secs_f64()
                ),
                sigma_position_m = %format!("{:?}", target.sigma_position),
                target_x = f4(target.position.x),
                target_y = f4(target.position.y),
                target_z = f4(target.position.z),
                rail_commanded_m = f4(rail_commanded_m),
                aim_commanded_rad = f4(aim_commanded_rad),
                alignment_duration_secs = f4(alignment.duration_secs),
                post_alignment_hold_secs = pingpong_bot::defaults::POST_ALIGNMENT_HOLD_SECS,
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
    let ready = initialize_pose_attempt(hardware, arm, true)?;
    log_startup_racket_geometry(arm, &ready);
    return Ok(ready);
}

/// 시작 실측 관절을 FK에 넣어 라켓 장착 모델과 자로 잰 실물 기준의 차이를 기록한다.
/// 여기서 `model_*`은 엔코더를 읽은 뒤의 **모델 계산값**이지 별도 자세 센서값이 아니다.
fn log_startup_racket_geometry(arm: &Arm, pose: &pingpong_bot::robot::Pose) {
    let Some(racket) = arm.forward_kinematics_with_rail(pose.rail_x, &pose.joints) else {
        warn!("초기 라켓 기하 진단 FK 실패");
        return;
    };
    let Some(wrist) = arm
        .joint_origins_world(pose.rail_x, &pose.joints)
        .and_then(|origins| origins.last().copied())
    else {
        warn!("초기 라켓 기하 진단 손목축 계산 실패");
        return;
    };

    let [w, x, y, z] = racket.orientation;
    let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z));
    // RacketPose 계약: local Y=블레이드 장축, local Z=면 법선.
    let axis_x = rotation * nalgebra::Vector3::x();
    let blade_axis = rotation * nalgebra::Vector3::y();
    let axis_normal = rotation * nalgebra::Vector3::z();
    let angle_from_vertical_deg = blade_axis.z.abs().clamp(-1.0, 1.0).acos().to_degrees();
    let face_above_horizontal_deg = racket
        .normal
        .z
        .atan2(racket.normal.x.hypot(racket.normal.y))
        .to_degrees();
    let vertical_half_extent = axis_x.z.abs() * pingpong_bot::constants::geometry::RACKET_HALF_X
        + blade_axis.z.abs() * pingpong_bot::constants::geometry::RACKET_HALF_Y
        + axis_normal.z.abs() * pingpong_bot::constants::geometry::RACKET_HALF_Z;
    let table_z = pingpong_bot::constants::table::SURFACE_Z;
    let model_wrist_above_table_m = wrist.z - table_z;
    let model_reference_above_table_m = racket.position.z - table_z;
    let model_lowest_above_table_m = model_reference_above_table_m - vertical_half_extent;
    let model_highest_above_table_m = model_reference_above_table_m + vertical_half_extent;
    let joints_measured_deg: Vec<f64> = pose
        .joints
        .values
        .iter()
        .map(|angle| angle.to_degrees())
        .collect();

    info!(
        rail_measured_m = f4(pose.rail_x),
        joints_measured_rad = %format!("{:?}", pose.joints.values),
        joints_measured_deg = %format!("{joints_measured_deg:?}"),
        model_wrist_above_table_m = f4(model_wrist_above_table_m),
        bench_wrist_above_table_m = f4(BENCH_WRIST_ABOVE_TABLE_M),
        wrist_model_minus_bench_m = f4(model_wrist_above_table_m - BENCH_WRIST_ABOVE_TABLE_M),
        model_racket_reference_above_table_m = f4(model_reference_above_table_m),
        model_racket_lowest_above_table_m = f4(model_lowest_above_table_m),
        bench_racket_lowest_above_table_m = f4(BENCH_RACKET_LOWEST_ABOVE_TABLE_M),
        lowest_model_minus_bench_m = f4(model_lowest_above_table_m - BENCH_RACKET_LOWEST_ABOVE_TABLE_M),
        model_racket_highest_above_table_m = f4(model_highest_above_table_m),
        bench_handle_end_above_table_m = f4(BENCH_HANDLE_END_ABOVE_TABLE_M),
        model_axis_from_vertical_deg = f2(angle_from_vertical_deg),
        bench_axis_from_vertical_deg = f2(BENCH_RACKET_AXIS_FROM_VERTICAL_DEG),
        axis_model_minus_bench_deg = f2(angle_from_vertical_deg - BENCH_RACKET_AXIS_FROM_VERTICAL_DEG),
        model_face_above_horizontal_deg = f2(face_above_horizontal_deg),
        model_collision_blade_length_m = f4(2.0 * pingpong_bot::constants::geometry::RACKET_HALF_Y),
        bench_total_racket_length_m = f4(BENCH_RACKET_TOTAL_LENGTH_M),
        "초기 라켓 기하 검증 — 모델 계산값과 벤치 실측 비교"
    );
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
    let ready_joints = arm.default_joints.clone();
    let ready_rail_x = arm
        .rail
        .as_ref()
        .map_or(measured.rail_x, |rail| rail.default_x());
    let trajectories =
        plan_neutral_return_segments(arm, &measured, ready_rail_x).map_err(MoveError::Plan)?;
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
            let correction =
                Planner::move_to(arm, &pose, compensated, pose.rail_x).map_err(MoveError::Plan)?;
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
            let (worst_joint_index, worst_joint_error_rad) = joint_errors
                .iter()
                .enumerate()
                .max_by(|(_, left), (_, right)| {
                    left.abs()
                        .partial_cmp(&right.abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map_or((0, 0.0), |(index, error)| (index, *error));
            let worst_motor_id = mapping.config().motor_ids[worst_joint_index];
            warn!(
                rail_commanded_m = f4(ready_rail_x),
                rail_measured_m = f4(pose.rail_x),
                rail_commanded_minus_measured_m = f4(ready_rail_x - pose.rail_x),
                joints_commanded = %format!("{:?}", ready_joints.values),
                joints_measured = %format!("{:?}", pose.joints.values),
                joints_commanded_minus_measured = %format!("{joint_errors:?}"),
                worst_joint_index,
                worst_motor_id,
                worst_joint_error_deg = worst_joint_error_rad.to_degrees(),
                "시작 팔 자세 실측 수렴 실패 — 모터별 Torque·Error·Goal·Present 진단 확인"
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
                worst_joint_index,
                worst_motor_id,
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

/// 시작 자세 초기화와 공 제어 후 복귀·수동 테스트 컨트롤이 같은 전체축 이동을 쓴다.
fn move_to_ready(hardware: &mut dyn Hardware, arm: &Arm, rail_x: f64) -> Result<(), MoveError> {
    let start = hardware.read_pose().map_err(MoveError::Hardware)?;
    let trajectories =
        plan_neutral_return_segments(arm, &start, rail_x).map_err(MoveError::Plan)?;
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

/// 존 변경(있다면) → 준비 자세 이동 → latch·상태 초기화 → 이벤트 발행까지 한 번에 한다.
/// `Wait`/`SetZone`은 idle일 때만 호출부가 부르고, `ResetPosition`은 즉시 부른다.
fn apply_test_control(
    control: TestControl,
    hardware: &mut dyn Hardware,
    arm: &Arm,
    home_rail_x: &mut f64,
    current_zone: &mut TestZone,
    latch: &mut CommandLatch,
    state: &mut BallControlState,
    sim_tx: Option<&Sender<SimUpdate>>,
    event_tx: &Sender<RuntimeEvent>,
) -> Result<(), MoveError> {
    let (target_zone, target_rail_x) = if let TestControl::SetZone(zone) = control
        && let Some(rail) = arm.rail
    {
        (zone, zone.rail_x(rail))
    } else {
        (*current_zone, *home_rail_x)
    };
    move_to_ready(hardware, arm, target_rail_x)?;
    *current_zone = target_zone;
    *home_rail_x = target_rail_x;
    *latch = CommandLatch::default();
    *state = BallControlState::Idle;
    if let Ok(pose) = hardware.read_pose()
        && let Some(sim_tx) = sim_tx
    {
        let _ = sim_tx.try_send(SimUpdate {
            pose: Some(PoseMsg::from(&pose)),
            ..SimUpdate::default()
        });
    }
    info!(
        control = ?control,
        zone = ?current_zone,
        home_rail_x = f4(*home_rail_x),
        "테스트 컨트롤 적용 — 준비 자세 복귀"
    );
    let _ = event_tx.send(RuntimeEvent::ControlState {
        state: ControlStateSnapshot::Idle,
    });
    let _ = event_tx.send(RuntimeEvent::TestZoneChanged {
        zone: *current_zone,
        home_rail_x: *home_rail_x,
    });
    return Ok(());
}

/// 직접 복귀가 테이블을 스치면 안전한 상승 중간 자세를 거치는 2구간을 찾는다.
/// 모든 구간은 실행 전에 속도·토크·테이블 충돌 검사를 통과해야 한다. 목표
/// 레일 x는 호출측이 고른다 — 시작 자세 초기화는 항상 `rail.default_x()`를,
/// 수동 테스트 컨트롤은 존 선택에 따른 값을 넘긴다.
fn plan_neutral_return_segments(
    arm: &Arm,
    start: &pingpong_bot::robot::Pose,
    rail_x: f64,
) -> Result<Vec<pingpong_bot::robot::motion::Trajectory>, DomainError> {
    match Planner::return_to_center_at(arm, start, rail_x) {
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
        match Planner::return_to_center_at(arm, &lifted_pose, rail_x) {
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
    StartupAlignmentTimeout {
        max_joint_error_rad: f64,
        worst_joint_index: usize,
        worst_motor_id: u8,
    },
}

impl std::fmt::Display for MoveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::Hardware(error) => write!(f, "{error}"),
            Self::Plan(error) => write!(f, "{error}"),
            Self::StartupAlignmentTimeout {
                max_joint_error_rad,
                worst_joint_index,
                worst_motor_id,
            } => write!(
                f,
                "시작 팔 자세가 10초 안에 수렴하지 않음: j{worst_joint_index} / Dynamixel ID {worst_motor_id}, 최대 관절 오차 {:+.2}°. 충돌 후 혼 위치·링크 체결 및 해당 ID의 Torque/Error 진단을 확인하세요",
                max_joint_error_rad.to_degrees(),
            ),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::Point3;
    use pingpong_bot::robot::control::{HitTarget, PredictionStage};
    use pingpong_bot::robot::{Joints, Pose};
    use pingpong_bot::vision::{State as VisionState, Track, Trajectory as VisionTrajectory};

    fn vision_state(t_secs: f64, y: f64) -> VisionState {
        return VisionState {
            t: Duration::from_secs_f64(t_secs),
            position: Point3::new(0.72, y, 0.94),
            velocity: nalgebra::Vector3::new(0.0, -4.0, 0.0),
            sigma_position: nalgebra::Vector3::repeat(0.02),
            sigma_velocity: nalgebra::Vector3::repeat(0.1),
            spin: None,
        };
    }

    fn vision_request(age: Duration) -> CommitRequest {
        return CommitRequest {
            trajectory: VisionTrajectory {
                seq: 9,
                origin: Instant::now() - Duration::from_secs(1),
                measured: Track(vec![vision_state(0.20, 0.80)]),
                predicted: Track(vec![
                    vision_state(0.20, 0.80),
                    vision_state(0.35, 0.50),
                    vision_state(0.45, 0.35),
                    vision_state(0.55, 0.20),
                    vision_state(0.65, 0.05),
                ]),
            },
            at: Instant::now() - age,
        };
    }

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
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        // 2026-08-05 실기 로그에서 직접 복귀가 테이블을 2mm 관통했던 실측 자세.
        let start = Pose::new(
            1.258_578,
            Joints::from_slice(&[1.264_000_169, -0.423_378_697, 0.115_048_559, -0.550_699_103]),
        );

        let segments = plan_neutral_return_segments(&robot.arm, &start, rail.default_x())
            .expect("직접 또는 상승 중간 자세를 거쳐 안전하게 복귀");
        assert!(!segments.is_empty());
        assert!(segments.len() <= 2);
    }

    #[test]
    fn delayed_vision_request_is_advanced_instead_of_dropped() {
        let request = vision_request(Duration::from_millis(80));
        let target = select_alignment_target(&request, motion::InterceptWindow::default())
            .expect("80ms 지연 요청도 미래 궤적으로 보정");

        assert!((target.position.y - 0.20).abs() < 0.031);
        assert!(target.t > Duration::from_millis(280));
    }

    #[test]
    fn vision_request_is_rejected_only_after_prediction_has_ended() {
        let request = vision_request(Duration::from_secs(1));
        assert!(select_alignment_target(&request, motion::InterceptWindow::default()).is_err());
    }

    #[test]
    fn each_vision_track_is_sent_only_once() {
        let mut latch = CommandLatch::default();
        assert!(latch.should_send(1));
        latch.mark_finished();
        assert!(!latch.should_send(1));
        assert!(latch.should_send(2));
    }

    #[test]
    fn new_track_resets_latch() {
        let mut latch = CommandLatch::default();
        assert!(latch.should_send(1));
        latch.mark_finished();
        assert!(latch.should_send(2));
    }

    #[test]
    fn aligned_track_is_permanently_blocked_even_after_returning_to_idle() {
        let mut latch = CommandLatch::default();
        assert!(latch.should_send(3));
        latch.mark_finished();
        assert!(!latch.should_send(3));

        assert!(latch.should_send(4));
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

    #[test]
    fn apply_test_control_set_zone_moves_home_clears_latch_and_emits_zone_event() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.default_x(), robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        latch.should_send(9);
        latch.mark_finished();
        let mut state = BallControlState::Aligning {
            track_seq: 9,
            return_due_at: Instant::now(),
            measurement: PendingAlignmentMeasurement {
                track_seq: 9,
                rail_commanded_m: rail.default_x(),
                joints_commanded: robot.arm.default_joints.clone(),
            },
        };
        let mut home_rail_x = rail.default_x();
        let mut current_zone = TestZone::Center;
        let (event_tx, event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::SetZone(TestZone::Left),
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply set zone");

        assert_eq!(current_zone, TestZone::Left);
        assert!((home_rail_x - TestZone::Left.rail_x(rail)).abs() < 1e-9);
        assert!(matches!(state, BallControlState::Idle));
        assert!(latch.should_send(9));
        assert!((hardware.pose.rail_x - TestZone::Left.rail_x(rail)).abs() < 1e-6);

        let events: Vec<_> = event_rx.try_iter().collect();
        assert!(events.iter().any(|event| matches!(
            event,
            RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Idle
            }
        )));
        assert!(events.iter().any(|event| matches!(
            event,
            RuntimeEvent::TestZoneChanged {
                zone: TestZone::Left,
                ..
            }
        )));
    }

    #[test]
    fn apply_test_control_wait_keeps_current_zone() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.x_max, robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        let mut state = BallControlState::Idle;
        let mut home_rail_x = rail.x_max;
        let mut current_zone = TestZone::Right;
        let (event_tx, _event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::Wait,
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply wait");

        assert_eq!(current_zone, TestZone::Right);
        assert!((home_rail_x - rail.x_max).abs() < 1e-9);
    }
}
