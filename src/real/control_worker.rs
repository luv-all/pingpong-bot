//! 실물 2단계 단순 제어 워커.
//!
//! 시작할 때만 기존 센터 궤적으로 레일과 4축 Dynamixel을 기본 자세에 둔다.
//! 이후 공 하나당 1차·2차 예측을 각각 한 번만 소비한다. 각 예측 위치의 x로
//! 레일을 옮기고, 2차에서만 라켓을 잡은 마지막 관절에 작은 시험 동작을 준다.
//! IK·스윙 계획·팔로스루는 실행하지 않고, 2차 동작 후에만 중앙으로 복귀한다.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::Point3;
use pingpong_bot::error::HwError;
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::Arm;
use pingpong_bot::robot::control::{HitTargetSelector, PredictionStage};
use pingpong_bot::robot::motion::{self, InterceptWindow};
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
/// 예측 충돌 직후 라켓 시험 동작이 보인 다음 중앙 복귀를 시작하는 여유.
const RETURN_AFTER_IMPACT_MARGIN: Duration = Duration::from_millis(150);
/// 관전 sim 명령은 공 표시 큐가 차 있어도 잠시 기다려 유실을 피한다.
const SIM_CONTROL_SEND_TIMEOUT: Duration = Duration::from_millis(20);

/// 실제 타격용이 아닌, 응답 확인용 손목 이동량.
const TEST_STROKE_RAD: f64 = 15.0_f64.to_radians();
const MIN_RAIL_COMMAND_SECS: f64 = 0.05;
const MAX_RAIL_COMMAND_SECS: f64 = 0.30;
/// 실기 로그의 발사기 오른쪽 정상 표본에서 예측이 공보다 평균적으로
/// 도메인 +X 쪽에 치우쳤다. 라켓 반폭 안에 들어오도록 -4 cm만 보정한다.
const LAUNCHER_RIGHT_RAIL_BIAS_M: f64 = 0.04;

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
    wrist_target_rad: f64,
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
        let ready_wrist = arm
            .default_joints
            .values
            .last()
            .copied()
            .unwrap_or_else(|| pose.joints.values.last().copied().unwrap_or(0.0));
        let selector = HitTargetSelector::new(intercept.y_min, intercept.y_max)
            .expect("검증된 목표 선택 구간");
        let rail_center = arm
            .rail
            .as_ref()
            .map(|rail| rail.default_x())
            .unwrap_or(pose.rail_x);

        let mut sim_pose = pose.clone();
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
        info!(
            wrist_ready_rad = f2(ready_wrist),
            "2단계 단순 제어 준비 — 공마다 레일 최대 2회, 손목축만 추가 제어"
        );

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
                match hardware.read_rail_x_m() {
                    Ok(actual_rail_x) => {
                        // 현재 단순 제어는 나머지 관절을 기본 자세로 두고
                        // 마지막 손목축만 바꾸므로, 해당 명령 자세의 FK를 같이 찍는다.
                        let mut racket_joints = arm.default_joints.clone();
                        if let Some(racket) = racket_joints.values.last_mut() {
                            *racket = diagnostic.wrist_target_rad;
                        }
                        let racket_center = arm
                            .forward_kinematics_with_rail(actual_rail_x, &racket_joints)
                            .map(|pose| pose.position);
                        info!(
                            command = diagnostic.command,
                            shot = diagnostic.shot,
                            stage = ?diagnostic.stage,
                            predicted_ball_x = f2(diagnostic.predicted_ball.x),
                            predicted_ball_y = f2(diagnostic.predicted_ball.y),
                            predicted_ball_z = f2(diagnostic.predicted_ball.z),
                            rail_target_x = f2(diagnostic.rail_target_x),
                            actual_rail_x = f2(actual_rail_x),
                            rail_error_m = f2(actual_rail_x - diagnostic.rail_target_x),
                            racket_center_x = racket_center.map(|point| f2(point.x)),
                            predicted_to_racket_error_m = racket_center
                                .map(|point| f2(point.x - diagnostic.predicted_ball.x)),
                            wrist_target_deg = f2(diagnostic.wrist_target_rad.to_degrees()),
                            elapsed_secs = f2(sampled_at.duration_since(diagnostic.issued_at).as_secs_f64()),
                            deadline_late_ms = f2(
                                sampled_at
                                    .saturating_duration_since(diagnostic.expected_at)
                                    .as_secs_f64()
                                    * 1e3,
                            ),
                            "공 로봇 라인 도달 예정 시각의 실제 레일·라켓 위치"
                        );
                    }
                    Err(error) => warn!(
                        %error,
                        command = diagnostic.command,
                        rail_target_x = f2(diagnostic.rail_target_x),
                        "공 로봇 라인 도달 예정 시각의 실제 레일 위치 읽기 실패"
                    ),
                }
            }

            if recovery_deadline.is_some_and(|deadline| Instant::now() >= deadline) {
                let return_reason = if latch.refined_sent {
                    "2차 동작 완료"
                } else {
                    "2차 예측 없음·예상 도달 시각 경과"
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
                sim_pose = pingpong_bot::robot::Pose::new(rail_center, arm.default_joints.clone());
                if let Some(sim_tx) = &sim_tx {
                    send_sim_control(
                        sim_tx,
                        SimUpdate {
                            pose: Some(PoseMsg::from(&sim_pose)),
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

            let target = match selector.select(&request.trajectory) {
                Ok(target) => target,
                Err(_) => continue,
            };
            let elapsed = request.trajectory.reference_time.elapsed().as_secs_f64();
            let remaining = target.time_secs - elapsed;
            if !remaining.is_finite() || remaining <= 0.0 {
                continue;
            }

            // 레일은 1차·2차 예측 위치의 x만 사용한다. 중간 관측은 추종하지 않는다.
            // 범위 밖 예측을 끝점으로 clamp하면 잘못된 공을 정상 제어로
            // 보이게 하고 레일을 끝까지 급출발시킨다. 이 경우는 명령을 보내지 않는다.
            if let Some(rail) = arm.rail.as_ref()
                && (target.position.x < rail.x_min || target.position.x > rail.x_max)
            {
                warn!(
                    requested_stage = ?requested_stage,
                    sent_stage = ?stage,
                    prediction_x = f2(target.position.x),
                    rail_x_min = f2(rail.x_min),
                    rail_x_max = f2(rail.x_max),
                    target_side_launcher = launcher_side(target.position.x - rail_center),
                    "레일 범위 밖 예측 제외 — 레일 명령 안 보냄"
                );
                continue;
            }
            let rail_x = corrected_rail_x(target.position.x, rail_center);
            if let Some(rail) = arm.rail.as_ref()
                && (rail_x < rail.x_min || rail_x > rail.x_max)
            {
                warn!(
                    requested_stage = ?requested_stage,
                    sent_stage = ?stage,
                    prediction_x = f2(target.position.x),
                    corrected_rail_x = f2(rail_x),
                    rail_x_min = f2(rail.x_min),
                    rail_x_max = f2(rail.x_max),
                    "오른쪽 보정 후 레일 범위 밖 예측 제외"
                );
                continue;
            }
            let previous_target_x = sim_pose.rail_x;
            let wrist = test_wrist_goal(ready_wrist, stage);
            let duration = remaining.clamp(MIN_RAIL_COMMAND_SECS, MAX_RAIL_COMMAND_SECS);
            if let Err(error) = hardware.command_rail_and_racket(rail_x, wrist, duration) {
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq,
                    reason: format!("레일·라켓 2단계 명령 실패: {error}"),
                });
                break;
            }

            // 실기와 동일한 레일+라켓 목표와 소요 시간을 관전 시뮬에도 보낸다.
            // 나머지 세 Dynamixel 관절은 현재 기본 자세를 그대로 유지한다.
            let mut sim_target_joints = sim_pose.joints.clone();
            if let Some(racket) = sim_target_joints.values.last_mut() {
                *racket = wrist;
            }
            let zero_velocity = vec![0.0; sim_target_joints.values.len()];
            let sim_motion = motion::Trajectory::new(
                sim_pose.joints.clone(),
                sim_target_joints.clone(),
                zero_velocity.clone(),
                zero_velocity,
                duration,
                motion::Rail {
                    start: sim_pose.rail_x,
                    end: rail_x,
                    start_velocity: 0.0,
                    end_velocity: 0.0,
                },
            );
            if let Some(sim_tx) = &sim_tx {
                send_sim_control(
                    sim_tx,
                    SimUpdate {
                        swing: Some(SwingMsg::from_trajectory(&sim_motion)),
                        ..SimUpdate::default()
                    },
                    "레일·라켓 명령",
                );
            }
            sim_pose = pingpong_bot::robot::Pose::new(rail_x, sim_target_joints);
            latch.mark_sent(stage);
            let issued_at = Instant::now();
            last_command = Some(issued_at);
            command_seq = command_seq.saturating_add(1);
            let expected_at = issued_at + Duration::from_secs_f64(remaining.max(0.0));
            impact_diagnostic = Some(PendingImpactDiagnostic {
                shot: shot_seq,
                command: command_seq,
                stage,
                predicted_ball: target.position,
                rail_target_x: rail_x,
                wrist_target_rad: wrist,
                issued_at,
                expected_at,
            });
            recovery_deadline = Some(expected_at + RETURN_AFTER_IMPACT_MARGIN);

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
                prediction_x = f2(target.position.x),
                prediction_y = f2(target.position.y),
                prediction_z = f2(target.position.z),
                ball_to_prediction_dx = f2(target.position.x - request.ball_x),
                rail_x = f2(rail_x),
                launcher_right_bias_m = f2(rail_x - target.position.x),
                target_side_launcher = launcher_side(rail_x - rail_center),
                command_delta_x = f2(rail_x - previous_target_x),
                command_direction_launcher = launcher_side(rail_x - previous_target_x),
                wrist_deg = f2(wrist.to_degrees()),
                remaining_secs = f2(remaining),
                "2단계 X 좌표 추적 명령"
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

/// 도메인 +X는 로봇 시점 오른쪽이므로, 발사기에서 보면 왼쪽이다.
fn launcher_side(domain_delta_x: f64) -> &'static str {
    if domain_delta_x > 1e-6 {
        return "왼쪽";
    }
    if domain_delta_x < -1e-6 {
        return "오른쪽";
    }
    return "중앙/정지";
}

/// 발사기 오른쪽(도메인 X가 중앙보다 작은 쪽)만 실측 보정한다.
fn corrected_rail_x(prediction_x: f64, rail_center: f64) -> f64 {
    if prediction_x < rail_center {
        return prediction_x - LAUNCHER_RIGHT_RAIL_BIAS_M;
    }
    return prediction_x;
}

/// 1차에는 기본각을 유지하고, 2차에만 손목을 15° 전진시킨다.
fn test_wrist_goal(ready: f64, stage: PredictionStage) -> f64 {
    return match stage {
        PredictionStage::Provisional => ready,
        PredictionStage::Refined => ready + TEST_STROKE_RAD,
    };
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

    #[test]
    fn only_refined_prediction_moves_racket() {
        let ready = -0.7;
        assert_eq!(test_wrist_goal(ready, PredictionStage::Provisional), ready);
        assert!(
            (test_wrist_goal(ready, PredictionStage::Refined) - (ready + TEST_STROKE_RAD)).abs()
                < 1e-12
        );
    }

    #[test]
    fn domain_x_direction_is_reported_from_launcher_view() {
        assert_eq!(launcher_side(0.1), "왼쪽");
        assert_eq!(launcher_side(-0.1), "오른쪽");
        assert_eq!(launcher_side(0.0), "중앙/정지");
    }

    #[test]
    fn launcher_right_prediction_gets_four_centimeter_bias() {
        assert!((corrected_rail_x(0.50, 0.705) - 0.46).abs() < 1e-12);
        assert_eq!(corrected_rail_x(0.90, 0.705), 0.90);
    }
}
