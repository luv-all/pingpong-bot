//! 실물 공 위치·방향 정렬·후속 팔 보정 제어 워커.
//!
//! `run`이 워커 시작 전에 레일과 4축 Dynamixel을 최초 중립 자세에 둔다.
//! 이후 공 하나당 보정된 접촉점에 라켓을 정지 정렬하고, 예상 타격
//! 0.25초 전에 백스윙 없는 고정 관절 스윙을 시작한다. 스윙이 불가하면 그 동작만 생략하고
//! 정렬 자세를 유지한 뒤 현재 모드의 준비 자세로 복귀한다.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::error::{DomainError, HwError};
use pingpong_bot::hardware::dynamixel::{DynamixelConfig, MotorMapping};
use pingpong_bot::hardware::{AppliedRailRacketCommand, Hardware};
use pingpong_bot::robot::control::{
    DirectControlCommand, DirectControlMeasurement, PredictionStage,
};
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
const FIXED_SWING_LEAD: Duration = Duration::from_millis(250);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const BUSY_POLL: Duration = Duration::from_millis(5);
const VERIFY_POLL_PERIOD: Duration = Duration::from_millis(20);
const VERIFY_STABLE_SAMPLES: u8 = 2;
const MAX_CONSECUTIVE_MISSES: u8 = 3;
const RAIL_ERROR_WARN_M: f64 = 0.020;
const AIM_ERROR_WARN_RAD: f64 = 3.0_f64.to_radians();
const STARTUP_SETTLE_TIMEOUT: Duration = Duration::from_secs(10);
// 시작 얼라인은 관절별 1° 이내가 5회 연속 측정돼야 완료로 본다.
const STARTUP_JOINT_TOLERANCE_RAD: f64 = 1.0_f64.to_radians();
const STARTUP_TRIM_DELAY: Duration = Duration::from_secs(1);
const STARTUP_MAX_TRIM_ATTEMPTS: u8 = 6;
const STARTUP_MAX_TRIM_STEP_RAD: f64 = 5.0_f64.to_radians();
/// 실측 잔여 오차를 한 번에 전부 더해 j2가 목표를 지나치는 것을 막는 보정 이득.
const STARTUP_TRIM_GAIN: f64 = 0.70;
/// 엔코더 한두 틱의 진동은 보정하지 않는다.
const STARTUP_TRIM_MIN_ERROR_RAD: f64 = 0.25_f64.to_radians();
/// 전역 IK가 다른 팔 접힘 가지를 고르며 듀얼 MX-64 축을 급회전시키는 것을 막는다.
const MAX_DUAL_BASE_STEP_RAD: f64 = 25.0_f64.to_radians();
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
    primary_sent: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RefinedAction {
    /// 본 예측 첫 명령: 레일과 팔을 함께 계산·이동한다.
    PrimaryRailAndArm,
    /// 같은 공의 후속 갱신: 이미 계산된 레일 위치에서 팔만 미세 보정한다.
    ArmCorrection,
}

impl CommandLatch {
    fn next_action(&mut self, track_seq: u64, refined_ready: bool) -> Option<RefinedAction> {
        if self.track_seq != Some(track_seq) {
            *self = Self::default();
            self.track_seq = Some(track_seq);
        }
        if !refined_ready {
            return None;
        }
        return Some(if self.primary_sent {
            RefinedAction::ArmCorrection
        } else {
            RefinedAction::PrimaryRailAndArm
        });
    }

    fn mark_primary_sent(&mut self) {
        self.primary_sent = true;
    }
}

/// 이 기준을 만족한 본 예측만 실제 제어 명령에 사용한다.
fn refined_prediction_ready(request: &CommitRequest) -> bool {
    let Some(last) = request.trajectory.measured.last() else {
        return false;
    };
    let params = pingpong_bot::defaults::EstimatorParams::default();
    let position_limit = nalgebra::Vector3::repeat(params.max_impact_sigma);
    let velocity_limit = nalgebra::Vector3::repeat(params.max_impact_sigma / params.max_lead);
    return last.sigma_position < position_limit && last.sigma_velocity < velocity_limit;
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
///
/// `Waiting`은 스윙(정렬→유지→중립 복귀) 완료 뒤 항상 들어가는 상태다 —
/// `TestControl::Next`(`n` 키)가 올 때까지 새 공을 무시한다. `w` 키로
/// 언제든(정렬 중이라도) 수동으로도 들어갈 수 있다.
enum BallControlState {
    Idle,
    Aligning {
        swing_due_at: Instant,
        swing_attempted: bool,
        return_due_at: Instant,
        measurement: PendingAlignmentMeasurement,
    },
    Waiting,
}

impl BallControlState {
    fn active_track_seq(&self) -> Option<u64> {
        return match self {
            Self::Idle | Self::Waiting => None,
            Self::Aligning { measurement, .. } => Some(measurement.track_seq),
        };
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
        let mut zone_filter: Option<TestZone> = None;

        if let Some(sim_tx) = &sim_tx {
            let _ = sim_tx.try_send(SimUpdate {
                pose: Some(PoseMsg::from(&pose)),
                ..SimUpdate::default()
            });
        }
        let mut cached_idle_pose = Some(pose.clone());
        let _ = event_tx.send(RuntimeEvent::Ready { pose });
        let _ = event_tx.send(RuntimeEvent::ControlState {
            state: ControlStateSnapshot::Idle,
        });
        let _ = event_tx.send(RuntimeEvent::TestZoneChanged {
            zone: current_zone,
            home_rail_x,
            filtering: false,
        });
        info!("공 위치·방향 정렬 준비 — 예상 타격 0.25초 전 고정 관절 스윙");

        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
        let mut state = BallControlState::Idle;
        // Dynamixel 이동 중에 도착한 같은 공의 본 예측은 최신 하나만 유지하고,
        // 현재 관절 이동이 끝나자마자 다음 미세 보정으로 적용한다.
        let mut pending_refined: Option<CommitRequest> = None;
        let mut consecutive_misses: u8 = 0;
        let mut pending_test_control: Option<TestControl> = None;
        let mut last_filtered_track_seq: Option<u64> = None;
        let mut last_waiting_ignored_track_seq: Option<u64> = None;

        'control: while !shutdown.is_down() {
            while let Ok(control) = test_control_rx.try_recv() {
                match control {
                    TestControl::ResetPosition | TestControl::Wait => {
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
                        pending_refined = None;
                        consecutive_misses = 0;
                        if apply_immediate_control(
                            control,
                            hardware.as_mut(),
                            &arm,
                            &mut home_rail_x,
                            &mut current_zone,
                            &mut zone_filter,
                            &mut latch,
                            &mut state,
                            sim_tx.as_ref(),
                            &event_tx,
                            &mut cached_idle_pose,
                        )
                        .is_break()
                        {
                            break 'control;
                        }
                    }
                    TestControl::Next => {
                        if matches!(state, BallControlState::Waiting) {
                            pending_verification = None;
                            pending_refined = None;
                            consecutive_misses = 0;
                            if apply_immediate_control(
                                TestControl::Next,
                                hardware.as_mut(),
                                &arm,
                                &mut home_rail_x,
                                &mut current_zone,
                                &mut zone_filter,
                                &mut latch,
                                &mut state,
                                sim_tx.as_ref(),
                                &event_tx,
                                &mut cached_idle_pose,
                            )
                            .is_break()
                            {
                                break 'control;
                            }
                        } else {
                            debug!("대기 상태가 아닐 때 'n' 입력 — 무시");
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
            let due_swing = match &state {
                BallControlState::Aligning {
                    swing_due_at,
                    swing_attempted,
                    measurement,
                    ..
                } if !swing_attempted && Instant::now() >= *swing_due_at => {
                    Some((measurement.track_seq, *swing_due_at))
                }
                BallControlState::Idle
                | BallControlState::Waiting
                | BallControlState::Aligning { .. } => None,
            };
            if let Some((track_seq, swing_due_at)) = due_swing {
                if let BallControlState::Aligning {
                    swing_attempted, ..
                } = &mut state
                {
                    // 한 공에 대해 성공·실패와 관계없이 한 번만 시도한다.
                    *swing_attempted = true;
                }
                if hardware.is_busy() {
                    warn!(
                        track_seq,
                        late_ms = f2(swing_due_at.elapsed().as_secs_f64() * 1e3),
                        "타격 0.25초 전에 정렬 동작이 아직 진행 중 — 고정 스윙만 생략"
                    );
                } else {
                    match hardware.read_pose() {
                        Ok(swing_start) => match Planner::fixed_joint_swing(&arm, &swing_start) {
                            Ok(planned) => {
                                let swing = &planned.trajectory;
                                match hardware.command_joints(swing) {
                                    Ok(()) => {
                                        if let BallControlState::Aligning { measurement, .. } =
                                            &mut state
                                        {
                                            measurement.rail_commanded_m = swing_start.rail_x;
                                            measurement.joints_commanded =
                                                swing.follow_through.clone();
                                        }
                                        info!(
                                            track_seq,
                                            scheduled_lead_secs = FIXED_SWING_LEAD.as_secs_f64(),
                                            start_late_ms = f2(swing_due_at.elapsed().as_secs_f64() * 1e3),
                                            swing_duration_secs = f4(swing.duration_secs),
                                            joints_start = %format!("{:?}", swing.start.values),
                                            joints_impact = %format!("{:?}", swing.end.values),
                                            joints_follow_through = %format!("{:?}", swing.follow_through.values),
                                            skipped_joint_indices = ?planned.skipped_joint_indices,
                                            "백스윙 없는 고정 관절 스윙 시작"
                                        );
                                    }
                                    Err(error) => warn!(
                                        track_seq,
                                        %error,
                                        "고정 스윙 명령 실패 — 스윙만 생략"
                                    ),
                                }
                            }
                            Err(error) => warn!(
                                track_seq,
                                %error,
                                "고정 스윙 계획 불가 — 스윙만 생략"
                            ),
                        },
                        Err(error) => warn!(
                            track_seq,
                            %error,
                            "고정 스윙 직전 포즈 읽기 실패 — 스윙만 생략"
                        ),
                    }
                }
            }
            let due_for_return = match &state {
                BallControlState::Aligning { return_due_at, .. } => {
                    Instant::now() >= *return_due_at
                }
                BallControlState::Idle | BallControlState::Waiting => false,
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
                    &mut zone_filter,
                    &mut latch,
                    &mut state,
                    sim_tx.as_ref(),
                    &event_tx,
                ) {
                    Ok(()) => cached_idle_pose = hardware.read_pose().ok(),
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
                                "타격 완료 후 현재 자세 유지"
                            );
                            cached_idle_pose = Some(measured.clone());
                            if let Some(sim_tx) = &sim_tx {
                                let _ = sim_tx.try_send(SimUpdate {
                                    pose: Some(PoseMsg::from(&measured)),
                                    ..SimUpdate::default()
                                });
                            }
                        }
                        Err(error) => {
                            cached_idle_pose = None;
                            warn!(%error, "타격 완료 후 유지 포즈 읽기 실패")
                        }
                    }
                }
                info!("준비 자세 복귀 없이 타격 자세 유지 — n 대기");
                // 타격 후 어떤 레일·관절 복귀 명령도 보내지 않는다.
                // 운영자가 `n`을 누르면 이 자세에서 바로 다음 공을 받는다.
                state = BallControlState::Waiting;
                pending_refined = None;
                let _ = event_tx.send(RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Waiting,
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
                && let BallControlState::Aligning {
                    swing_due_at,
                    swing_attempted,
                    return_due_at,
                    ..
                } = &state
            {
                let swing_wait = if !swing_attempted {
                    swing_due_at.saturating_duration_since(now)
                } else {
                    RECV_TIMEOUT
                };
                let return_wait = if *return_due_at <= now && hardware.is_busy() {
                    BUSY_POLL
                } else {
                    return_due_at.saturating_duration_since(now)
                };
                timeout = timeout.min(swing_wait).min(return_wait);
            }
            let can_apply_latest = pending_refined.is_some()
                && !hardware.is_busy()
                && last_command.is_none_or(|at| at.elapsed() >= COMMAND_THROTTLE);
            let request = if can_apply_latest {
                pending_refined.take().expect("확인한 최신 본 예측")
            } else {
                match rx.recv_timeout(timeout) {
                    Ok(request) => request,
                    Err(RecvTimeoutError::Disconnected) => break,
                    Err(RecvTimeoutError::Timeout) => continue,
                }
            };
            let track_seq = request.track_seq();
            // 대기(`Waiting`) 중에는 `n`을 누르기 전까지 어떤 공도 명령하지
            // 않는다 — 같은 track을 반복 로그하지 않도록 1회만 남긴다.
            if matches!(state, BallControlState::Waiting) {
                if last_waiting_ignored_track_seq != Some(track_seq) {
                    info!(track_seq, "대기 중(n 대기) — 공 명령 생략");
                    last_waiting_ignored_track_seq = Some(track_seq);
                }
                continue;
            }
            if matches!(
                state,
                BallControlState::Aligning {
                    swing_attempted: true,
                    ..
                }
            ) {
                // 스윙을 시작하거나 시간을 놓친 후에는 새 예측이 관절 명령을
                // 덮어써서 타격 동작을 중단하지 못하게 한다.
                continue;
            }
            // 한 공의 정렬·유지가 끝나기 전에 검출기가 만든 새 잡음 track이
            // latch와 복귀 상태를 덮어쓰지 못하게 한다.
            if state
                .active_track_seq()
                .is_some_and(|active| active != track_seq)
            {
                continue;
            }
            let refined_ready = refined_prediction_ready(&request);
            let Some(action) = latch.next_action(track_seq, refined_ready) else {
                continue;
            };
            if hardware.is_busy() || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
            {
                pending_refined = Some(request);
                continue;
            }

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
            let corrected_target_x =
                target.position.x - pingpong_bot::defaults::ALIGNMENT_LAUNCHER_RIGHT_OFFSET_M;
            if let Some(zone) = zone_filter
                && let Some(rail) = arm.rail
                && !zone.contains_x(rail, corrected_target_x)
            {
                if last_filtered_track_seq != Some(track_seq) {
                    let (zone_min_m, zone_max_m) = zone.bounds(rail);
                    info!(
                        track_seq,
                        zone = zone.label(),
                        corrected_target_x = f4(corrected_target_x),
                        zone_min_m = f4(zone_min_m),
                        zone_max_m = f4(zone_max_m),
                        "선택한 제어 구간 밖의 공 — 명령 생략"
                    );
                    last_filtered_track_seq = Some(track_seq);
                }
                continue;
            }
            last_filtered_track_seq = None;
            // 준비 자세 복귀 직후 읽어 둔 실측 자세를 재사용한다. 토크가 유지되는
            // 대기 중에는 자세가 바뀌지 않으므로 느린 4-ID 직렬 읽기를 없앤다.
            let (start, start_pose_source) = if matches!(state, BallControlState::Idle) {
                match cached_idle_pose.take() {
                    Some(pose) => (pose, "cached_ready"),
                    None => match hardware.read_pose() {
                        Ok(pose) => (pose, "measured"),
                        Err(error) => {
                            warn!(track_seq, %error, "본 예측 명령 직전 포즈 읽기 실패");
                            continue;
                        }
                    },
                }
            } else {
                match hardware.read_pose() {
                    Ok(pose) => (pose, "measured"),
                    Err(error) => {
                        warn!(track_seq, %error, "실시간 팔 보정 직전 포즈 읽기 실패");
                        continue;
                    }
                }
            };
            if let Some(previous) = pending_verification.take() {
                log_verification(&previous, &start, "superseded", false);
            }
            let issued_at = Instant::now();
            let alignment = match action {
                RefinedAction::PrimaryRailAndArm => {
                    Planner::ball_alignment(&arm, &start, target.position)
                }
                RefinedAction::ArmCorrection => {
                    Planner::ball_alignment_fixed_rail(&arm, &start, target.position)
                }
            };
            let alignment = match alignment {
                Ok(alignment) => alignment,
                Err(error) => {
                    let _ = event_tx.send(RuntimeEvent::Failed {
                        track_seq: Some(track_seq),
                        reason: format!("본 예측 {action:?} 정렬 계획 불가: {error}"),
                    });
                    continue;
                }
            };
            let dual_base_step_rad = alignment
                .follow_through
                .values
                .first()
                .zip(start.joints.values.first())
                .map_or(0.0, |(target, measured)| target - measured);
            if dual_base_step_rad.abs() > MAX_DUAL_BASE_STEP_RAD {
                let _ = event_tx.send(RuntimeEvent::Failed {
                    track_seq: Some(track_seq),
                    reason: format!(
                        "본 예측 정렬 듀얼 MX-64 급회전 차단: 현재 대비 {:+.1}° (허용 ±25°) — 다음 공에서 재시도",
                        dual_base_step_rad.to_degrees(),
                    ),
                });
                continue;
            }
            let rail_commanded_m = alignment.rail.end;
            let corrected_target_position = pingpong_bot::Point3::new(
                target.position.x - pingpong_bot::defaults::ALIGNMENT_LAUNCHER_RIGHT_OFFSET_M,
                target.position.y,
                target.position.z,
            );
            let aim_commanded_rad = alignment
                .end
                .values
                .get(pingpong_bot::robot::control::DIRECT_AIM_JOINT_INDEX)
                .copied()
                .unwrap_or(0.0);
            let command_result = match action {
                RefinedAction::PrimaryRailAndArm => hardware.command(&alignment),
                RefinedAction::ArmCorrection => hardware.command_joints(&alignment),
            };
            if let Err(error) = command_result {
                let _ = event_tx.send(RuntimeEvent::Failed {
                    track_seq: Some(track_seq),
                    reason: format!("위치·방향 정렬 명령 실패: {error}"),
                });
                break;
            }
            if action == RefinedAction::PrimaryRailAndArm {
                latch.mark_primary_sent();
            }
            last_command = Some(issued_at);
            let _ = event_tx.send(RuntimeEvent::Commanded {
                track_seq,
                target: corrected_target_position,
                rail_x: rail_commanded_m,
                aim_rad: aim_commanded_rad,
            });
            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    target: Some(corrected_target_position),
                    ..SimUpdate::default()
                });
            }

            let predicted_arrival_at = request.trajectory.origin + target.t;
            let swing_due_at = predicted_arrival_at
                .checked_sub(FIXED_SWING_LEAD)
                .unwrap_or(issued_at);
            let return_due_at = predicted_arrival_at
                + Duration::from_secs_f64(pingpong_bot::defaults::POST_ALIGNMENT_HOLD_SECS);
            state = BallControlState::Aligning {
                swing_due_at,
                swing_attempted: false,
                return_due_at,
                measurement: PendingAlignmentMeasurement {
                    track_seq,
                    rail_commanded_m,
                    joints_commanded: alignment.follow_through.clone(),
                },
            };
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
                stage = ?PredictionStage::Refined,
                start_pose_source,
                request_age_secs = f4(request.age_secs()),
                target_time_secs = f4(target.t.as_secs_f64()),
                predicted_arrival_in_secs = f4(
                    predicted_arrival_at
                        .saturating_duration_since(Instant::now())
                        .as_secs_f64()
                ),
                sigma_position_m = %format!("{:?}", target.sigma_position),
                vision_target_x = f4(target.position.x),
                corrected_target_x = f4(corrected_target_position.x),
                target_y = f4(target.position.y),
                target_z = f4(target.position.z),
                rail_commanded_m = f4(rail_commanded_m),
                control_action = ?action,
                aim_commanded_rad = f4(aim_commanded_rad),
                alignment_duration_secs = f4(alignment.duration_secs),
                dual_base_step_deg = f2(dual_base_step_rad.to_degrees()),
                post_alignment_hold_secs = pingpong_bot::defaults::POST_ALIGNMENT_HOLD_SECS,
                fixed_swing_lead_secs = FIXED_SWING_LEAD.as_secs_f64(),
                joints_commanded = %format!("{:?}", alignment.follow_through.values),
                "본 예측 레일·팔 정렬/팔 실시간 미세 보정 시작 — 고정 스윙 예약"
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

/// 직전 모터 목표에 `commanded - measured`를 누적한다.
fn accumulate_startup_trim_goal(
    arm: &Arm,
    compensated_goal_values: &mut [f64],
    joint_errors: &[f64],
) -> Vec<f64> {
    let mut incremental_correction_deg = Vec::with_capacity(joint_errors.len());
    for (index, error) in joint_errors.iter().copied().enumerate() {
        let correction = if error.abs() > STARTUP_TRIM_MIN_ERROR_RAD {
            (error * STARTUP_TRIM_GAIN).clamp(-STARTUP_MAX_TRIM_STEP_RAD, STARTUP_MAX_TRIM_STEP_RAD)
        } else {
            0.0
        };
        let next_goal = compensated_goal_values[index] + correction;
        compensated_goal_values[index] = arm
            .joint_limit(index)
            .map_or(next_goal, |limit| next_goal.clamp(limit.min, limit.max));
        incremental_correction_deg.push(correction.to_degrees());
    }
    return incremental_correction_deg;
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
    hardware
        .verify_coupled_joints()
        .map_err(MoveError::Hardware)?;
    hardware
        .arm_joint_limit_escape(&measured.joints)
        .map_err(MoveError::Hardware)?;
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
    hardware
        .verify_coupled_joints()
        .map_err(MoveError::Hardware)?;
    // executor 종료는 마지막 Goal Position을 보냈다는 뜻일 뿐, 모터가 실제로
    // 도착했다는 뜻은 아니다. 실측이 준비 자세에 연속 두 번 들어올 때까지 기다린다.
    let settle_started = Instant::now();
    let settle_deadline = settle_started + STARTUP_SETTLE_TIMEOUT;
    let mut next_trim_at = settle_started + STARTUP_TRIM_DELAY;
    let mut trim_attempts = 0_u8;
    let mut stable_samples = 0_u8;
    // 매 보정을 ready 목표에서 다시 시작하면 중력·유격 편차가 다시
    // 돌아온다. 직전 모터 목표에 실측 오차를 누적해 진짜 폐루프로 보정한다.
    let mut compensated_goal_values = ready_joints.values.clone();
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
            let incremental_correction_deg =
                accumulate_startup_trim_goal(arm, &mut compensated_goal_values, &joint_errors);
            let compensated = Joints::from_slice(&compensated_goal_values);
            let cumulative_correction_deg: Vec<f64> = compensated_goal_values
                .iter()
                .zip(&ready_joints.values)
                .map(|(commanded, target)| (commanded - target).to_degrees())
                .collect();
            let correction =
                Planner::move_to(arm, &pose, compensated, pose.rail_x).map_err(MoveError::Plan)?;
            trim_attempts += 1;
            info!(
                attempt = trim_attempts,
                incremental_correction_deg = %format!("{incremental_correction_deg:?}"),
                cumulative_correction_deg = %format!("{cumulative_correction_deg:?}"),
                measured_error_deg = %format!("{:?}", joint_errors.iter().map(|error| error.to_degrees()).collect::<Vec<_>>()),
                "시작 팔 자세 잔여 오차 누적 폐루프 보정"
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
    hardware
        .verify_coupled_joints()
        .map_err(MoveError::Hardware)?;
    hardware
        .arm_joint_limit_escape(&start.joints)
        .map_err(MoveError::Hardware)?;
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
    hardware
        .verify_coupled_joints()
        .map_err(MoveError::Hardware)?;
    return Ok(());
}

/// 존 변경(있다면) → 준비 자세 이동 → latch·상태 초기화 → 이벤트 발행까지 한 번에 한다.
/// `SetZone`/`DefaultMode`는 idle일 때만 호출부가 부르고, `ResetPosition`/`Wait`/`Next`는
/// 즉시(`apply_immediate_control` 경유) 부른다.
///
/// 컨트롤 종류에 따라 적용 뒤 상태가 갈린다: `ResetPosition`/`Next`는 항상
/// `Idle`(공을 받는 상태)로, `Wait`는 항상 `Waiting`(`n` 대기)으로 만든다.
/// `SetZone`/`DefaultMode`는 존만 바꿀 뿐 — 호출 시점이 `Waiting`이었으면
/// `Waiting`을 유지하고, 아니면(`Idle`/`Aligning`) `Idle`로 정리한다.
fn apply_test_control(
    control: TestControl,
    hardware: &mut dyn Hardware,
    arm: &Arm,
    home_rail_x: &mut f64,
    current_zone: &mut TestZone,
    zone_filter: &mut Option<TestZone>,
    latch: &mut CommandLatch,
    state: &mut BallControlState,
    sim_tx: Option<&Sender<SimUpdate>>,
    event_tx: &Sender<RuntimeEvent>,
) -> Result<(), MoveError> {
    let (target_zone, target_rail_x, target_filter) = match (control, arm.rail) {
        (TestControl::SetZone(zone), Some(rail)) => (zone, zone.rail_x(rail), Some(zone)),
        (TestControl::DefaultMode, Some(rail)) => (TestZone::Center, rail.default_x(), None),
        _ => (*current_zone, *home_rail_x, *zone_filter),
    };
    let resulting_state = match control {
        TestControl::ResetPosition | TestControl::Next => BallControlState::Idle,
        TestControl::Wait => BallControlState::Waiting,
        TestControl::SetZone(_) | TestControl::DefaultMode => {
            if matches!(state, BallControlState::Waiting) {
                BallControlState::Waiting
            } else {
                BallControlState::Idle
            }
        }
    };
    // `Next`는 타격 후 자세를 그대로 유지하고 제어만 재개한다.
    // 명시적인 리셋·존 변경·기본 모드만 준비 자세로 이동한다.
    if !matches!(control, TestControl::Next) {
        move_to_ready(hardware, arm, target_rail_x)?;
    }
    *current_zone = target_zone;
    *zone_filter = target_filter;
    *home_rail_x = target_rail_x;
    *latch = CommandLatch::default();
    *state = resulting_state;
    let control_state_snapshot = if matches!(state, BallControlState::Waiting) {
        ControlStateSnapshot::Waiting
    } else {
        ControlStateSnapshot::Idle
    };
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
        zone_filter = ?zone_filter,
        home_rail_x = f4(*home_rail_x),
        resulting_state = ?control_state_snapshot,
        held_impact_pose = matches!(control, TestControl::Next),
        "테스트 컨트롤 적용"
    );
    let _ = event_tx.send(RuntimeEvent::ControlState {
        state: control_state_snapshot,
    });
    let _ = event_tx.send(RuntimeEvent::TestZoneChanged {
        zone: *current_zone,
        home_rail_x: *home_rail_x,
        filtering: zone_filter.is_some(),
    });
    return Ok(());
}

/// `ResetPosition`/`Wait`/`Next`처럼 idle 대기 없이 즉시 적용하는 컨트롤의
/// 공통 결과 처리 — 성공하면 idle 포즈를 캐시하고, 실패하면 이벤트를 보낸
/// 뒤 세션을 끊어야 하는지(`ControlFlow::Break`) 계속해도 되는지
/// (`ControlFlow::Continue`)를 알려준다.
fn apply_immediate_control(
    control: TestControl,
    hardware: &mut dyn Hardware,
    arm: &Arm,
    home_rail_x: &mut f64,
    current_zone: &mut TestZone,
    zone_filter: &mut Option<TestZone>,
    latch: &mut CommandLatch,
    state: &mut BallControlState,
    sim_tx: Option<&Sender<SimUpdate>>,
    event_tx: &Sender<RuntimeEvent>,
    cached_idle_pose: &mut Option<pingpong_bot::robot::Pose>,
) -> std::ops::ControlFlow<()> {
    return match apply_test_control(
        control,
        hardware,
        arm,
        home_rail_x,
        current_zone,
        zone_filter,
        latch,
        state,
        sim_tx,
        event_tx,
    ) {
        Ok(()) => {
            *cached_idle_pose = hardware.read_pose().ok();
            std::ops::ControlFlow::Continue(())
        }
        Err(MoveError::Hardware(error)) => {
            let _ = event_tx.send(RuntimeEvent::Failed {
                track_seq: latch.track_seq,
                reason: format!("수동 컨트롤 적용 중 하드웨어 오류: {error}"),
            });
            std::ops::ControlFlow::Break(())
        }
        Err(error @ MoveError::Plan(_))
        | Err(error @ MoveError::StartupAlignmentTimeout { .. }) => {
            warn!(%error, "수동 컨트롤 적용 중 준비 자세 계획 실패 — 세션은 유지");
            let _ = event_tx.send(RuntimeEvent::Failed {
                track_seq: latch.track_seq,
                reason: format!("수동 컨트롤 적용 중 준비 자세 계획 실패: {error}"),
            });
            *state = BallControlState::Idle;
            let _ = event_tx.send(RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Idle,
            });
            std::ops::ControlFlow::Continue(())
        }
    };
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
    let planning_start = clamp_small_joint_limit_overshoot(arm, start);
    let direct_error = match Planner::return_to_center_at_speed_ratio(
        arm,
        &planning_start,
        rail_x,
        pingpong_bot::defaults::HOME_RETURN_SPEED_RATIO,
    ) {
        Ok(direct) => return Ok(vec![direct]),
        Err(error) => error,
    };

    let racket = arm
        .forward_kinematics_with_rail(planning_start.rail_x, &planning_start.joints)
        .ok_or_else(|| {
            DomainError::InfeasibleSwing(
                pingpong_bot::error::SwingPlanError::InverseKinematicsNoSolution {
                    target_x: start.rail_x,
                    target_y: 0.0,
                    target_z: 0.0,
                },
            )
        })?;
    // 직접 복귀가 테이블 관통뿐 아니라 관절 한계·토크 한계로 실패해도 상승
    // 중간 자세를 시도한다. 극단 예측 자세에서는 정지→정지 단일 quintic이
    // 중간에 한계를 넘지만, 위로 먼저 접은 뒤에는 준비 자세 복귀가 가능할 수 있다.
    let mut last_error = Some(direct_error);
    for lift_m in [0.03, 0.06, 0.10, 0.15] {
        let lifted_target = pingpong_bot::Point3::new(
            racket.position.x,
            racket.position.y,
            racket.position.z + lift_m,
        );
        let lifted_joints = match arm.rail.as_ref() {
            Some(rail) => arm.inverse_kinematics_with_rail(
                rail,
                planning_start.rail_x,
                lifted_target,
                Some(&planning_start.joints),
            ),
            None => arm.inverse_kinematics_near(lifted_target, Some(&planning_start.joints)),
        };
        let lifted_joints = match lifted_joints {
            Ok(joints) => joints,
            Err(error) => {
                last_error = Some(DomainError::InfeasibleSwing(error));
                continue;
            }
        };
        let lift = match Planner::move_to_at_speed_ratio(
            arm,
            &planning_start,
            lifted_joints,
            planning_start.rail_x,
            pingpong_bot::defaults::HOME_RETURN_SPEED_RATIO,
        ) {
            Ok(trajectory) => trajectory,
            Err(error) => {
                last_error = Some(error);
                continue;
            }
        };
        let lifted_pose =
            pingpong_bot::robot::Pose::new(lift.follow_through_rail_x, lift.follow_through.clone());
        match Planner::return_to_center_at_speed_ratio(
            arm,
            &lifted_pose,
            rail_x,
            pingpong_bot::defaults::HOME_RETURN_SPEED_RATIO,
        ) {
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

/// Dynamixel 양자화·탄성 때문에 관절이 URDF 한계를 아주 조금 넘으면 시작점
/// 자체가 불가능하다고 판정되어 모든 복귀가 막힌다. 1° 이내 초과만 계획 좌표에서
/// 경계로 붙이고, 그보다 큰 초과는 그대로 두어 안전 검사가 실패하게 한다.
fn clamp_small_joint_limit_overshoot(
    arm: &Arm,
    start: &pingpong_bot::robot::Pose,
) -> pingpong_bot::robot::Pose {
    const MAX_RECOVERABLE_OVERSHOOT_RAD: f64 = 1.0_f64.to_radians();
    let mut joints = start.joints.clone();
    for (index, angle) in joints.values.iter_mut().enumerate() {
        let Some(limit) = arm.joint_limit(index) else {
            continue;
        };
        let clamped = angle.clamp(limit.min, limit.max);
        if (*angle - clamped).abs() <= MAX_RECOVERABLE_OVERSHOOT_RAD {
            *angle = clamped;
        }
    }
    return pingpong_bot::robot::Pose::new(start.rail_x, joints);
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
    fn startup_trim_adds_damped_measured_error_to_previous_motor_goal() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let mut goal = robot.arm.default_joints.values.clone();
        let original = goal.clone();
        let errors = [0.0, 0.0, -2.0_f64.to_radians(), 0.1_f64.to_radians()];

        accumulate_startup_trim_goal(&robot.arm, &mut goal, &errors);
        accumulate_startup_trim_goal(&robot.arm, &mut goal, &errors);

        assert!((goal[2] - (original[2] - 2.8_f64.to_radians())).abs() < 1e-12);
        assert_eq!(goal[3], original[3], "0.25° 미만 진동은 무시");
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
    fn extreme_logged_alignment_pose_has_a_safe_ready_return() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let start = Pose::new(
            0.1114,
            Joints::from_slice(&[0.5031457, 0.5246214, -1.3284274, 0.2914563]),
        );
        let rail = robot.arm.rail.expect("rail");

        let segments = plan_neutral_return_segments(&robot.arm, &start, rail.default_x())
            .expect("극단 정렬 자세에서도 중간 자세를 거쳐 복귀");

        assert_eq!(
            segments.last().expect("ready segment").follow_through,
            robot.arm.default_joints
        );
    }

    #[test]
    fn plan_neutral_return_segments_is_slower_than_full_speed_return() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let start = Pose::new(rail.x_max, robot.arm.default_joints.clone());

        let home_segments = plan_neutral_return_segments(&robot.arm, &start, rail.x_min)
            .expect("홈 포지션 복귀 계획");
        let full_speed = Planner::return_to_center_at(&robot.arm, &start, rail.x_min)
            .expect("전속 복귀 계획(비교 기준)");

        let home_duration: f64 = home_segments
            .iter()
            .map(|segment| segment.duration_secs)
            .sum();
        assert!(
            home_duration > full_speed.duration_secs * 2.0,
            "home_duration={home_duration} full_speed={}",
            full_speed.duration_secs
        );
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
    fn each_vision_track_sends_primary_then_arm_corrections() {
        let mut latch = CommandLatch::default();
        assert_eq!(latch.next_action(1, false), None);
        assert_eq!(
            latch.next_action(1, true),
            Some(RefinedAction::PrimaryRailAndArm)
        );
        latch.mark_primary_sent();
        assert_eq!(
            latch.next_action(1, true),
            Some(RefinedAction::ArmCorrection)
        );
    }

    #[test]
    fn new_track_resets_latch() {
        let mut latch = CommandLatch::default();
        assert_eq!(
            latch.next_action(1, true),
            Some(RefinedAction::PrimaryRailAndArm)
        );
        latch.mark_primary_sent();
        assert_eq!(
            latch.next_action(2, true),
            Some(RefinedAction::PrimaryRailAndArm)
        );
    }

    #[test]
    fn refined_prediction_uses_existing_sigma_gate() {
        let refined = vision_request(Duration::ZERO);
        assert!(refined_prediction_ready(&refined));

        let mut provisional = vision_request(Duration::ZERO);
        provisional.trajectory.measured.0[0].sigma_position = nalgebra::Vector3::repeat(1.0);
        assert!(!refined_prediction_ready(&provisional));
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
    fn apply_test_control_set_zone_moves_home_clears_latch_and_emits_zone_event() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.default_x(), robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        assert_eq!(
            latch.next_action(9, true),
            Some(RefinedAction::PrimaryRailAndArm)
        );
        let mut state = BallControlState::Aligning {
            swing_due_at: Instant::now(),
            swing_attempted: false,
            return_due_at: Instant::now(),
            measurement: PendingAlignmentMeasurement {
                track_seq: 9,
                rail_commanded_m: rail.default_x(),
                joints_commanded: robot.arm.default_joints.clone(),
            },
        };
        let mut home_rail_x = rail.default_x();
        let mut current_zone = TestZone::Center;
        let mut zone_filter = None;
        let (event_tx, event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::SetZone(TestZone::Left),
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut zone_filter,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply set zone");

        assert_eq!(current_zone, TestZone::Left);
        assert_eq!(zone_filter, Some(TestZone::Left));
        assert!((home_rail_x - TestZone::Left.rail_x(rail)).abs() < 1e-9);
        assert!(matches!(state, BallControlState::Idle));
        assert_eq!(
            latch.next_action(9, true),
            Some(RefinedAction::PrimaryRailAndArm)
        );
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
    fn apply_test_control_wait_keeps_zone_and_enters_waiting() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.x_max, robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        let mut state = BallControlState::Idle;
        let mut home_rail_x = rail.x_max;
        let mut current_zone = TestZone::Right;
        let mut zone_filter = Some(TestZone::Right);
        let (event_tx, event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::Wait,
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut zone_filter,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply wait");

        assert_eq!(current_zone, TestZone::Right);
        assert_eq!(zone_filter, Some(TestZone::Right));
        assert!((home_rail_x - rail.x_max).abs() < 1e-9);
        assert!(matches!(state, BallControlState::Waiting));
        let events: Vec<_> = event_rx.try_iter().collect();
        assert!(events.iter().any(|event| matches!(
            event,
            RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Waiting
            }
        )));
    }

    #[test]
    fn apply_test_control_next_from_waiting_returns_to_idle() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let held_pose = Pose::new(rail.x_min, Joints::from_slice(&[0.35, -0.20, -0.45, 0.30]));
        let mut hardware = PoseApplyingHardware {
            pose: held_pose.clone(),
        };
        let mut latch = CommandLatch::default();
        let mut state = BallControlState::Waiting;
        let mut home_rail_x = rail.x_max;
        let mut current_zone = TestZone::Right;
        let mut zone_filter = Some(TestZone::Right);
        let (event_tx, event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::Next,
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut zone_filter,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply next");

        assert_eq!(current_zone, TestZone::Right, "next는 존을 바꾸지 않는다");
        assert!(matches!(state, BallControlState::Idle));
        assert_eq!(
            hardware.pose, held_pose,
            "next는 타격 후 레일·관절 자세를 바꾸지 않아야 한다"
        );
        let events: Vec<_> = event_rx.try_iter().collect();
        assert!(events.iter().any(|event| matches!(
            event,
            RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Idle
            }
        )));
    }

    #[test]
    fn apply_test_control_set_zone_from_waiting_preserves_waiting() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.default_x(), robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        let mut state = BallControlState::Waiting;
        let mut home_rail_x = rail.default_x();
        let mut current_zone = TestZone::Center;
        let mut zone_filter = None;
        let (event_tx, event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::SetZone(TestZone::Left),
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut zone_filter,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply set zone while waiting");

        assert_eq!(current_zone, TestZone::Left);
        assert_eq!(zone_filter, Some(TestZone::Left));
        assert!((home_rail_x - TestZone::Left.rail_x(rail)).abs() < 1e-9);
        assert!(
            matches!(state, BallControlState::Waiting),
            "존만 바뀌고 대기는 유지되어야 한다"
        );
        let events: Vec<_> = event_rx.try_iter().collect();
        assert!(events.iter().any(|event| matches!(
            event,
            RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Waiting
            }
        )));
    }

    #[test]
    fn apply_default_mode_clears_filter_and_returns_to_center() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let mut hardware = PoseApplyingHardware {
            pose: Pose::new(rail.x_min, robot.arm.default_joints.clone()),
        };
        let mut latch = CommandLatch::default();
        let mut state = BallControlState::Idle;
        let mut home_rail_x = rail.x_min;
        let mut current_zone = TestZone::Left;
        let mut zone_filter = Some(TestZone::Left);
        let (event_tx, _event_rx) = crossbeam_channel::unbounded();

        apply_test_control(
            TestControl::DefaultMode,
            &mut hardware,
            &robot.arm,
            &mut home_rail_x,
            &mut current_zone,
            &mut zone_filter,
            &mut latch,
            &mut state,
            None,
            &event_tx,
        )
        .expect("apply default mode");

        assert_eq!(current_zone, TestZone::Center);
        assert_eq!(zone_filter, None);
        assert!((home_rail_x - rail.default_x()).abs() < 1e-9);
    }

    /// `event_rx`를 소진하며 `predicate`를 만족하는 이벤트가 `timeout` 안에
    /// 오면 `true`, 그 안에 안 오면(채널 disconnect 포함) `false`.
    ///
    /// 못 찾은 경우는 "이 시간 동안 그 이벤트가 한 번도 없었다"는 뜻이기도
    /// 해서, "아무 일도 없어야 한다" 종류의 확인에도 그대로 쓴다.
    fn wait_for_event(
        event_rx: &crossbeam_channel::Receiver<RuntimeEvent>,
        timeout: Duration,
        mut predicate: impl FnMut(&RuntimeEvent) -> bool,
    ) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return false;
            }
            match event_rx.recv_timeout(remaining) {
                Ok(event) if predicate(&event) => return true,
                Ok(_) => {}
                Err(_) => return false,
            }
        }
    }

    #[test]
    fn spawn_ignores_balls_while_waiting_and_resumes_after_next() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let hardware: Box<dyn Hardware> = Box::new(ReadCountingHardware {
            reads: 0,
            pose: Pose::new(rail.default_x(), robot.arm.default_joints.clone()),
        });

        let (commit_tx, commit_rx) = crossbeam_channel::unbounded();
        let (test_control_tx, test_control_rx) = crossbeam_channel::unbounded();
        let (event_tx, event_rx) = crossbeam_channel::unbounded();
        let (guard, shutdown) = crate::real::shutdown_channel();

        let handle = spawn(
            hardware,
            Arc::clone(&robot.arm),
            commit_rx,
            test_control_rx,
            None,
            event_tx,
            shutdown,
        );

        let generous = Duration::from_secs(3);

        commit_tx
            .send(vision_request(Duration::ZERO))
            .expect("보낼 수 있음");
        assert!(
            wait_for_event(&event_rx, generous, |event| matches!(
                event,
                RuntimeEvent::Commanded { .. }
            )),
            "첫 공은 명령돼야 한다"
        );
        assert!(
            wait_for_event(&event_rx, generous, |event| matches!(
                event,
                RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Waiting
                }
            )),
            "스윙(정렬→유지→복귀) 완료 후 대기 상태로 들어가야 한다"
        );

        commit_tx
            .send(vision_request(Duration::ZERO))
            .expect("보낼 수 있음");
        assert!(
            !wait_for_event(&event_rx, Duration::from_millis(400), |event| matches!(
                event,
                RuntimeEvent::Commanded { .. }
            )),
            "대기 중에는 두 번째 공을 명령하면 안 된다"
        );

        test_control_tx
            .send(TestControl::Next)
            .expect("보낼 수 있음");
        assert!(
            wait_for_event(&event_rx, generous, |event| matches!(
                event,
                RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Idle
                }
            )),
            "'n' 이후에는 다시 공을 받는 상태로 돌아와야 한다"
        );

        commit_tx
            .send(vision_request(Duration::ZERO))
            .expect("보낼 수 있음");
        assert!(
            wait_for_event(&event_rx, generous, |event| matches!(
                event,
                RuntimeEvent::Commanded { .. }
            )),
            "재개 후 세 번째 공은 다시 명령돼야 한다"
        );

        drop(guard);
        handle.join().expect("워커 스레드가 정상 종료해야 한다");
    }
}
