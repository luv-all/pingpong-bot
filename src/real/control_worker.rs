//! 실물 2단계 전체 자세 제어 워커.
//!
//! 1차·2차 공 궤적 예측을 sim과 공통인 [`PositionController`]에 넘겨
//! 레일 + 4축 역기구학 후보를 고르고 전체 quintic 계획을 실물에 전송한다.
//! 2차 예측은 진행 중인 1차 명령을 취소하고 실측 포즈에서 다시 계획한다.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::Point3;
use pingpong_bot::error::HwError;
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::Arm;
use pingpong_bot::robot::control::{
    HitTargetSelector, PositionControlError, PositionController, PredictionStage,
};
use pingpong_bot::robot::motion::InterceptWindow;
use tracing::{info, info_span, warn};

use super::fmt::f2;
use super::{CommitRequest, ControlStatus, PoseMsg, ShotEvent, Shutdown, SimUpdate, SwingMsg};

const MAX_REQUEST_AGE_SECS: f64 = 0.050;
const COMMAND_THROTTLE: Duration = Duration::from_millis(20);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
/// 요청이 이만큼 끊기면 다음 추적은 새 공으로 본다.
const NEW_BALL_REQUEST_GAP: Duration = Duration::from_millis(500);
/// 로봇 쪽으로 오던 공의 y가 다시 이만큼 증가하면 새 투구로 본다.
const NEW_BALL_Y_RISE_M: f64 = 0.25;
/// 예 타격 직후 중앙 복귀를 시작하는 여유.
const RETURN_AFTER_IMPACT_MARGIN: Duration = Duration::from_millis(150);
/// 관전 sim 명령은 공 표시 큐가 차 있어도 잠시 기다려 유실을 피한다.
const SIM_CONTROL_SEND_TIMEOUT: Duration = Duration::from_millis(20);

/// 1차 스트리밍 종료를 기다리는 최대 시간.
const CANCEL_WAIT: Duration = Duration::from_millis(100);

#[derive(Default)]
struct TwoStageLatch {
    provisional_sent: bool,
    refined_sent: bool,
    last_request_at: Option<Instant>,
    min_ball_y: Option<f64>,
}

struct PendingImpactDiagnostic {
    shot: u64,
    command: u64,
    stage: PredictionStage,
    predicted_ball: Point3,
    rail_target_x: f64,
    issued_at: Instant,
    expected_at: Instant,
}

impl TwoStageLatch {
    fn stage_to_send(
        &mut self,
        requested: PredictionStage,
        ball_y: f64,
        at: Instant,
    ) -> Option<PredictionStage> {
        let request_gap = self
            .last_request_at
            .is_some_and(|last| at.saturating_duration_since(last) >= NEW_BALL_REQUEST_GAP);
        let y_rose = ball_y.is_finite()
            && self
                .min_ball_y
                .is_some_and(|min| min.is_finite() && ball_y - min >= NEW_BALL_Y_RISE_M);
        if request_gap || y_rose {
            self.provisional_sent = false;
            self.refined_sent = false;
            self.min_ball_y = ball_y.is_finite().then_some(ball_y);
        } else if ball_y.is_finite() {
            self.min_ball_y = Some(self.min_ball_y.map_or(ball_y, |min| min.min(ball_y)));
        }
        self.last_request_at = Some(at);

        if !self.provisional_sent {
            return Some(requested);
        }
        if self.refined_sent {
            return None;
        }
        if requested == PredictionStage::Refined {
            return Some(PredictionStage::Refined);
        }
        return None;
    }

    fn mark_sent(&mut self, stage: PredictionStage) {
        match stage {
            PredictionStage::Provisional => self.provisional_sent = true,
            PredictionStage::Refined => self.refined_sent = true,
        }
    }
}

/// 제어 워커를 띄운다. 실제 장비 동작은 이 워커를 실기 PC에서 실행할 때만 발생한다.
pub fn spawn(
    mut hardware: Box<dyn Hardware>,
    arm: Arc<Arm>,
    intercept: InterceptWindow,
    home: bool,
    rx: Receiver<CommitRequest>,
    status_tx: Sender<ControlStatus>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
    shutdown: Shutdown,
) -> JoinHandle<()> {
    return thread::spawn(move || {
        let _span = info_span!("control").entered();

        if home {
            if let Err(error) = move_to_center(hardware.as_mut(), &arm) {
                warn!(%error, "초기 센터 정렬 실패 — 2단계 제어를 시작하지 않는다");
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq: 1,
                    reason: format!("초기 센터 정렬 실패: {error}"),
                });
                let _ = event_tx.send(ShotEvent::Done);
                return;
            }
            info!("리니어 중앙 정렬 + 로봇 기본 자세 완료");
        }

        let pose = match hardware.read_pose() {
            Ok(pose) => pose,
            Err(error) => {
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq: 1,
                    reason: format!("시작 포즈 읽기 실패: {error}"),
                });
                let _ = event_tx.send(ShotEvent::Done);
                return;
            }
        };
        let selector = HitTargetSelector::new(intercept.y_min, intercept.y_max)
            .expect("검증된 목표 선택 구간");
        let rail_center = arm
            .rail
            .as_ref()
            .map(|rail| rail.default_x())
            .unwrap_or(pose.rail_x);

        if let Some(sim_tx) = &sim_tx {
            send_sim_control(
                sim_tx,
                SimUpdate {
                    pose: Some(PoseMsg::from(&pose)),
                    ..SimUpdate::default()
                },
                "초기 포즈",
            );
        }
        let mut shot_seq: u64 = 1;
        let _ = event_tx.send(ShotEvent::Armed { shot_seq, pose });
        let _ = status_tx.send(ControlStatus::Ready { shot_seq });
        info!("2단계 전체 자세 제어 준비 — 레일 + 4축 IK 궤적");

        let mut latch = TwoStageLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut impact_diagnostic: Option<PendingImpactDiagnostic> = None;
        let mut recovery_deadline: Option<Instant> = None;
        let mut command_seq: u64 = 0;

        while !shutdown.is_down() {
            if impact_diagnostic
                .as_ref()
                .is_some_and(|diagnostic| Instant::now() >= diagnostic.expected_at)
                && let Some(diagnostic) = impact_diagnostic.take()
            {
                let sampled_at = Instant::now();
                match hardware.read_pose() {
                    Ok(actual_pose) => {
                        let racket_center = arm
                            .forward_kinematics_with_rail(actual_pose.rail_x, &actual_pose.joints)
                            .map(|pose| pose.position);
                        info!(
                            command = diagnostic.command,
                            shot = diagnostic.shot,
                            stage = ?diagnostic.stage,
                            predicted_ball_x = f2(diagnostic.predicted_ball.x),
                            predicted_ball_y = f2(diagnostic.predicted_ball.y),
                            predicted_ball_z = f2(diagnostic.predicted_ball.z),
                            rail_target_x = f2(diagnostic.rail_target_x),
                            actual_rail_x = f2(actual_pose.rail_x),
                            rail_error_m = f2(actual_pose.rail_x - diagnostic.rail_target_x),
                            racket_center_x = racket_center.map(|point| f2(point.x)),
                            racket_center_y = racket_center.map(|point| f2(point.y)),
                            racket_center_z = racket_center.map(|point| f2(point.z)),
                            predicted_to_racket_error_m = racket_center
                                .map(|point| f2((point - diagnostic.predicted_ball).norm())),
                            elapsed_secs = f2(sampled_at.duration_since(diagnostic.issued_at).as_secs_f64()),
                            deadline_late_ms = f2(
                                sampled_at
                                    .saturating_duration_since(diagnostic.expected_at)
                                    .as_secs_f64()
                                    * 1e3,
                            ),
                            "공 도달 예정 시각의 실제 레일·4관절 FK 위치"
                        );
                    }
                    Err(error) => warn!(
                        %error,
                        command = diagnostic.command,
                        rail_target_x = f2(diagnostic.rail_target_x),
                        "공 도달 예정 시각의 실제 포즈 읽기 실패"
                    ),
                }
            }

            if recovery_deadline.is_some_and(|deadline| Instant::now() >= deadline) {
                // 타격점 hold까지 포함한 궤적이 아직 재생 중이면 중앙
                // 명령으로 덮지 않는다. executor가 완주한 다음 복귀한다.
                if hardware.is_busy() {
                    recovery_deadline = Some(Instant::now() + Duration::from_millis(20));
                    continue;
                }
                let return_reason = if latch.refined_sent {
                    "2차 전체 자세 계획 완료"
                } else {
                    "1차 전체 자세 계획 완료·2차 예측 없음"
                };
                let _ = status_tx.send(ControlStatus::Recovering { shot_seq });
                if let Err(error) = move_to_center(hardware.as_mut(), &arm) {
                    warn!(%error, return_reason, "중앙 복귀 실패 — 제어 중단");
                    let _ = event_tx.send(ShotEvent::Failed {
                        shot_seq,
                        reason: format!("중앙 복귀 실패: {error}"),
                    });
                    break;
                }
                let center_pose =
                    pingpong_bot::robot::Pose::new(rail_center, arm.default_joints.clone());
                if let Some(sim_tx) = &sim_tx {
                    send_sim_control(
                        sim_tx,
                        SimUpdate {
                            pose: Some(PoseMsg::from(&center_pose)),
                            ..SimUpdate::default()
                        },
                        "중앙 복귀 포즈",
                    );
                }
                latch = TwoStageLatch::default();
                last_command = None;
                impact_diagnostic = None;
                recovery_deadline = None;
                shot_seq = shot_seq.saturating_add(1);
                let _ = status_tx.send(ControlStatus::Ready { shot_seq });
                info!(
                    shot = shot_seq,
                    return_reason, "리니어 중앙 + 로봇 기본 자세 재정렬 완료"
                );
                continue;
            }

            let now = Instant::now();
            let mut wait = recovery_deadline.map_or(RECV_TIMEOUT, |deadline| {
                deadline.saturating_duration_since(now).min(RECV_TIMEOUT)
            });
            if let Some(diagnostic) = &impact_diagnostic {
                wait = wait.min(diagnostic.expected_at.saturating_duration_since(now));
            }
            let request = match rx.recv_timeout(wait) {
                Ok(request) => request,
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            };
            let requested_stage = request.stage;
            let Some(stage) = latch.stage_to_send(requested_stage, request.ball_y, request.at)
            else {
                continue;
            };
            if request.age_secs() > MAX_REQUEST_AGE_SECS
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
            {
                continue;
            }

            // 2차 예측은 1차 임시 이동을 선점한다. executor가 실제로
            // 멈춘 다음 캐시된 관절각 + 실측 레일 포즈에서 다시 IK를 푼다.
            if hardware.is_busy() {
                if stage != PredictionStage::Refined {
                    continue;
                }
                hardware.cancel();
                let cancel_deadline = Instant::now() + CANCEL_WAIT;
                while hardware.is_busy() && Instant::now() < cancel_deadline {
                    thread::sleep(Duration::from_millis(2));
                }
                if hardware.is_busy() {
                    warn!(shot = shot_seq, "1차 계획 취소 시간 초과 — 2차 재계획 보류");
                    continue;
                }
            }

            let start = match hardware.read_pose() {
                Ok(pose) => pose,
                Err(error) => {
                    let _ = event_tx.send(ShotEvent::Failed {
                        shot_seq,
                        reason: format!("재계획 시작 포즈 읽기 실패: {error}"),
                    });
                    break;
                }
            };
            let planned =
                match PositionController::plan_best(&arm, &start, &request.trajectory, &selector) {
                    Ok(planned) => planned,
                    Err(error) => {
                        let reason = error.to_string();
                        match error {
                            PositionControlError::Unreachable(_) => {
                                let _ = event_tx.send(ShotEvent::Infeasible { shot_seq, reason });
                            }
                            _ => {
                                let _ = event_tx.send(ShotEvent::PlanFailed { shot_seq, reason });
                            }
                        }
                        continue;
                    }
                };
            let remaining = planned.target.time_secs
                - request.trajectory.reference_time.elapsed().as_secs_f64();
            if !remaining.is_finite() || remaining <= 0.0 {
                continue;
            }
            let trajectory = planned.trajectory;
            let rail_x = trajectory.rail.end;
            if let Err(error) = hardware.command(&trajectory) {
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq,
                    reason: format!("레일 + 4관절 궤적 명령 실패: {error}"),
                });
                break;
            }

            if let Some(sim_tx) = &sim_tx {
                send_sim_control(
                    sim_tx,
                    SimUpdate {
                        swing: Some(SwingMsg::from_trajectory(&trajectory)),
                        ..SimUpdate::default()
                    },
                    "레일 + 4관절 IK 명령",
                );
            }
            latch.mark_sent(stage);
            let issued_at = Instant::now();
            last_command = Some(issued_at);
            command_seq = command_seq.saturating_add(1);
            let expected_at = issued_at + Duration::from_secs_f64(remaining.max(0.0));
            impact_diagnostic = Some(PendingImpactDiagnostic {
                shot: shot_seq,
                command: command_seq,
                stage,
                predicted_ball: planned.target.position,
                rail_target_x: rail_x,
                issued_at,
                expected_at,
            });
            recovery_deadline = Some(expected_at + RETURN_AFTER_IMPACT_MARGIN);

            let _ = event_tx.send(ShotEvent::Committed {
                shot_seq,
                time_to_impact_secs: remaining,
                duration_secs: trajectory.duration_secs,
                impact: planned.target.position,
                rail_start: trajectory.rail.start,
                rail_end: trajectory.rail.end,
                peak_joint_speed: trajectory.peak_joint_speed(),
            });

            info!(
                shot = shot_seq,
                command = command_seq,
                requested_stage = ?requested_stage,
                sent_stage = ?stage,
                stage_source = if requested_stage == stage {
                    "추정기"
                } else {
                    "제어 단계 조정"
                },
                raw_ball_x = request
                    .raw_ball_x
                    .map(f2)
                    .unwrap_or_else(|| "없음".to_owned()),
                ekf_ball_x = f2(request.ball_x),
                ekf_ball_y = f2(request.ball_y),
                ekf_ball_vx = f2(request.ball_vx),
                prediction_x = f2(planned.target.position.x),
                prediction_y = f2(planned.target.position.y),
                prediction_z = f2(planned.target.position.z),
                ball_to_prediction_dx = f2(planned.target.position.x - request.ball_x),
                rail_start = f2(trajectory.rail.start),
                rail_end = f2(trajectory.rail.end),
                joint_goal = ?trajectory.end.values,
                peak_joint_speed = f2(trajectory.peak_joint_speed()),
                peak_rail_speed = f2(trajectory.peak_rail_speed()),
                remaining_secs = f2(remaining),
                "2단계 레일 + 4관절 IK 궤적 명령"
            );

            if stage == PredictionStage::Refined {
                // 진짜 2차가 들어오면 추정을 멈추고, 위에서 예상 도달
                // 시각이 되었을 때 중앙으로 복귀한다.
                let _ = status_tx.send(ControlStatus::Recovering { shot_seq });
            }
        }

        let _ = event_tx.send(ShotEvent::Done);
    });
}

/// 실물 제어 명령이 빈번한 공 표시 메시지에 밀려 조용히 사라지지 않게 한다.
/// sim이 닫혔거나 느린 경우에도 실물 제어는 멈추지 않도록 대기 시간을 짧게 제한한다.
fn send_sim_control(sim_tx: &Sender<SimUpdate>, update: SimUpdate, kind: &'static str) {
    if let Err(error) = sim_tx.send_timeout(update, SIM_CONTROL_SEND_TIMEOUT) {
        warn!(%error, kind, "실물 제어의 sim 반영 실패");
    }
}

/// 시작 시 레일을 정확히 중앙으로 보내고 4축을 기본 자세로 만든다.
/// 플래너나 추가 Dynamixel read에 의존하지 않는 실기 초기화 전용 경로다.
fn move_to_center(hardware: &mut dyn Hardware, arm: &Arm) -> Result<(), MoveError> {
    let rail_center = arm
        .rail
        .as_ref()
        .map(|rail| rail.default_x())
        .unwrap_or(0.0);
    return hardware
        .command_initial_pose(rail_center, &arm.default_joints)
        .map_err(MoveError::Hardware);
}

#[derive(Debug)]
enum MoveError {
    Hardware(HwError),
}

impl std::fmt::Display for MoveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return match self {
            Self::Hardware(error) => write!(f, "{error}"),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_prediction_stage_is_sent_only_once_per_ball() {
        let mut latch = TwoStageLatch::default();
        let now = Instant::now();
        assert_eq!(
            latch.stage_to_send(PredictionStage::Provisional, 2.0, now),
            Some(PredictionStage::Provisional)
        );
        latch.mark_sent(PredictionStage::Provisional);
        assert_eq!(
            latch.stage_to_send(PredictionStage::Provisional, 1.9, now),
            None
        );
        assert_eq!(
            latch.stage_to_send(PredictionStage::Refined, 1.8, now),
            Some(PredictionStage::Refined)
        );
        latch.mark_sent(PredictionStage::Refined);
        assert_eq!(
            latch.stage_to_send(PredictionStage::Refined, 1.7, now),
            None
        );
    }

    #[test]
    fn new_ball_y_rise_rearms_even_when_previous_refined_was_missing() {
        let mut latch = TwoStageLatch::default();
        let now = Instant::now();
        assert_eq!(
            latch.stage_to_send(PredictionStage::Provisional, 1.2, now),
            Some(PredictionStage::Provisional)
        );
        latch.mark_sent(PredictionStage::Provisional);

        assert_eq!(
            latch.stage_to_send(PredictionStage::Provisional, 2.0, now),
            Some(PredictionStage::Provisional)
        );
    }

    #[test]
    fn second_provisional_waits_for_actual_refined_stage() {
        let mut latch = TwoStageLatch::default();
        let now = Instant::now();
        assert_eq!(
            latch.stage_to_send(PredictionStage::Provisional, 1.5, now),
            Some(PredictionStage::Provisional)
        );
        latch.mark_sent(PredictionStage::Provisional);

        assert_eq!(
            latch.stage_to_send(PredictionStage::Provisional, 1.4, now),
            None
        );
        assert_eq!(
            latch.stage_to_send(PredictionStage::Refined, 1.3, now),
            Some(PredictionStage::Refined)
        );
    }
}
