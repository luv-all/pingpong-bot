//! 실물 2단계 단순 제어 워커.
//!
//! 시작할 때만 기존 센터 궤적으로 레일과 4축 Dynamixel을 기본 자세에 둔다.
//! 이후 공 하나당 1차·2차 예측을 각각 한 번만 소비한다. 각 예측 위치의 x로
//! 레일을 옮기고, 2차에서만 라켓을 잡은 마지막 관절에 작은 시험 동작을 준다.
//! IK·스윙 계획·팔로스루·자동 복귀는 실행하지 않는다.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::error::{DomainError, HwError};
use pingpong_bot::hardware::{AppliedRailRacketCommand, Hardware};
use pingpong_bot::robot::Arm;
use pingpong_bot::robot::control::{
    DIRECT_WRIST_JOINT_INDEX, DirectControlCommand, DirectControlMeasurement, DirectController,
    PredictionStage,
};
use pingpong_bot::robot::motion::{self, Planner};
use tracing::{debug, info, info_span, warn};

use super::fmt::{f2, f4};
use super::{CommitRequest, PoseMsg, RuntimeEvent, Shutdown, SimUpdate};

const MAX_REQUEST_AGE_SECS: f64 = 0.050;
const COMMAND_THROTTLE: Duration = Duration::from_millis(20);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const BUSY_POLL: Duration = Duration::from_millis(5);
const VERIFY_POLL_PERIOD: Duration = Duration::from_millis(20);
const VERIFY_TIMEOUT_AFTER_COMMAND: Duration = Duration::from_millis(500);
const VERIFY_STABLE_SAMPLES: u8 = 2;
const MAX_CONSECUTIVE_MISSES: u8 = 3;
const RAIL_ERROR_WARN_M: f64 = 0.020;
const WRIST_ERROR_WARN_RAD: f64 = 3.0_f64.to_radians();

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
}

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
    home: bool,
    rx: Receiver<CommitRequest>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<RuntimeEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
    return thread::spawn(move || {
        let _span = info_span!("control").entered();

        if home && let Err(error) = move_to_center(hardware.as_mut(), &arm) {
            warn!(%error, "초기 센터 이동 실패 — 현재 자세에서 2단계 제어를 시작한다");
        }

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
        let ready_wrist = arm
            .default_joints
            .values
            .get(DIRECT_WRIST_JOINT_INDEX)
            .copied()
            .or_else(|| pose.joints.values.get(DIRECT_WRIST_JOINT_INDEX).copied())
            .unwrap_or(0.0);
        let window = motion::InterceptWindow::default();
        let controller = DirectController::new(window.y_min, window.y_max, ready_wrist)
            .expect("기본 레일·손목 제어 설정");

        if let Some(sim_tx) = &sim_tx {
            let _ = sim_tx.try_send(SimUpdate {
                pose: Some(PoseMsg::from(&pose)),
                ..SimUpdate::default()
            });
        }
        let _ = event_tx.send(RuntimeEvent::Ready { pose });
        info!(
            wrist_ready_rad = f2(ready_wrist),
            "2단계 단순 제어 준비 — 공마다 레일 최대 2회, 손목축만 추가 제어"
        );

        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
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
                                "레일·손목 수렴 실패 {consecutive_misses}회 연속 — 제어 중단"
                            ),
                        });
                        break;
                    }
                }
                VerificationResult::Pending => {}
            }
            let timeout = pending_verification
                .as_ref()
                .map_or(RECV_TIMEOUT, |pending| {
                    pending
                        .next_check_at
                        .saturating_duration_since(Instant::now())
                        .min(RECV_TIMEOUT)
                });
            let request = match rx.recv_timeout(timeout) {
                Ok(request) => request,
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            };
            if !latch.should_send(request.track_seq, request.stage)
                || request.age_secs() > MAX_REQUEST_AGE_SECS
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
            {
                continue;
            }

            let elapsed = request.trajectory.reference_time.elapsed().as_secs_f64();
            let start = match hardware.read_pose() {
                Ok(pose) => pose,
                Err(error) => {
                    warn!(track_seq = request.track_seq, %error, "명령 직전 포즈 읽기 실패");
                    continue;
                }
            };
            let command =
                match controller.command(&arm, &start, &request.trajectory, request.stage, elapsed)
                {
                    Ok(command) => command,
                    Err(error) => {
                        debug!(track_seq = request.track_seq, %error, "레일·손목 명령 계산 생략");
                        continue;
                    }
                };
            if let Some(previous) = pending_verification.take() {
                log_verification(&previous, &start, "superseded", false);
            }
            let applied = match hardware.command_rail_and_racket(
                command.rail_x,
                command.wrist_rad,
                command.duration_secs,
            ) {
                Ok(applied) => applied,
                Err(error) => {
                    let _ = event_tx.send(RuntimeEvent::Failed {
                        track_seq: Some(request.track_seq),
                        reason: format!("레일·라켓 2단계 명령 실패: {error}"),
                    });
                    break;
                }
            };
            latch.mark_sent(request.stage);
            last_command = Some(Instant::now());
            let _ = event_tx.send(RuntimeEvent::Commanded {
                track_seq: request.track_seq,
                stage: request.stage,
                target: command.target.position,
                rail_x: applied.rail_m,
                wrist_rad: applied.wrist_rad,
            });

            let issued_at = Instant::now();
            pending_verification = Some(PendingVerification {
                track_seq: request.track_seq,
                command,
                applied,
                issued_at,
                next_check_at: issued_at + Duration::from_secs_f64(command.duration_secs),
                deadline: issued_at
                    + Duration::from_secs_f64(command.duration_secs)
                    + VERIFY_TIMEOUT_AFTER_COMMAND,
                stable_samples: 0,
            });

            info!(
                stage = ?request.stage,
                prediction_x = f2(command.target.position.x),
                prediction_y = f2(command.target.position.y),
                prediction_z = f2(command.target.position.z),
                rail_requested_m = f4(command.rail_x),
                rail_applied_m = f4(applied.rail_m),
                rail_sent = applied.rail_sent,
                wrist_requested_rad = f4(command.wrist_rad),
                wrist_applied_rad = f4(applied.wrist_rad),
                wrist_applied_deg = f2(applied.wrist_rad.to_degrees()),
                duration_secs = f2(command.duration_secs),
                "2단계 단순 제어 명령"
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
                "명령 후 레일·손목 재측정 타임아웃"
            );
            pending.take();
            return VerificationResult::Missed;
        }
    };
    let verification = pending.as_mut().expect("시간이 된 명령 측정");
    let Some(measurement) = DirectControlMeasurement::from_commanded(
        verification.applied.rail_m,
        verification.applied.wrist_rad,
        &pose,
    ) else {
        warn!(
            track_seq = verification.track_seq,
            measured_joint_count = pose.joints.values.len(),
            "명령 후 재측정에 손목축이 없음"
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
        && measurement.wrist_error_rad.abs() <= WRIST_ERROR_WARN_RAD;
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
        wrist_error_deg = f2(measurement.wrist_error_rad.to_degrees()),
        "레일·손목 수렴 대기"
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
        verification.applied.wrist_rad,
        pose,
    ) else {
        return;
    };
    let outside_tolerance = measurement.rail_error_m.abs() > RAIL_ERROR_WARN_M
        || measurement.wrist_error_rad.abs() > WRIST_ERROR_WARN_RAD;
    let log_measurement = || {
        (
            measurement.rail_commanded_m,
            measurement.rail_measured_m,
            measurement.rail_error_m,
            measurement.wrist_commanded_rad.to_degrees(),
            measurement.wrist_measured_rad.to_degrees(),
            measurement.wrist_error_rad.to_degrees(),
        )
    };
    let (
        rail_commanded_m,
        rail_measured_m,
        rail_error_m,
        wrist_commanded_deg,
        wrist_measured_deg,
        wrist_error_deg,
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
            wrist_commanded_rad = f4(measurement.wrist_commanded_rad),
            wrist_measured_rad = f4(measurement.wrist_measured_rad),
            wrist_commanded_minus_measured_rad = f4(measurement.wrist_error_rad),
            wrist_commanded_deg = f2(wrist_commanded_deg),
            wrist_measured_deg = f2(wrist_measured_deg),
            wrist_commanded_minus_measured_deg = f2(wrist_error_deg),
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
            wrist_commanded_rad = f4(measurement.wrist_commanded_rad),
            wrist_measured_rad = f4(measurement.wrist_measured_rad),
            wrist_commanded_minus_measured_rad = f4(measurement.wrist_error_rad),
            wrist_commanded_deg = f2(wrist_commanded_deg),
            wrist_measured_deg = f2(wrist_measured_deg),
            wrist_commanded_minus_measured_deg = f2(wrist_error_deg),
            "명령 후 제어 괴리 측정"
        );
    }
}

/// 시작 시에만 기존 전체축 센터 이동을 사용한다.
fn move_to_center(hardware: &mut dyn Hardware, arm: &Arm) -> Result<(), MoveError> {
    let start = hardware.read_pose().map_err(MoveError::Hardware)?;
    let trajectory = Planner::return_to_center(arm, &start).map_err(MoveError::Plan)?;
    hardware.command(&trajectory).map_err(MoveError::Hardware)?;
    while hardware.is_busy() {
        thread::sleep(BUSY_POLL);
    }
    return Ok(());
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
    fn due_command_needs_two_stable_readbacks() {
        let command = DirectControlCommand {
            stage: PredictionStage::Refined,
            target: HitTarget {
                position: Point3::new(0.30, 0.20, 0.25),
                incoming_velocity: nalgebra::Vector3::zeros(),
                time_secs: 0.2,
            },
            rail_x: 0.30,
            wrist_rad: -0.40,
            duration_secs: 0.1,
        };
        let mut hardware = ReadCountingHardware {
            reads: 0,
            pose: Pose::new(0.29, Joints::from_slice(&[0.0, 0.0, 0.0, -0.41])),
        };
        let mut pending = Some(PendingVerification {
            track_seq: 7,
            command,
            applied: AppliedRailRacketCommand {
                rail_m: 0.30,
                wrist_rad: -0.40,
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
}
