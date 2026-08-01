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
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::Arm;
use pingpong_bot::robot::control::{HitTargetSelector, PredictionStage};
use pingpong_bot::robot::motion::{self, Planner};
use tracing::{info, info_span, warn};

use super::fmt::f2;
use super::{CommitRequest, ControlStatus, PoseMsg, ShotEvent, Shutdown, SimUpdate};

const MAX_REQUEST_AGE_SECS: f64 = 0.050;
const COMMAND_THROTTLE: Duration = Duration::from_millis(20);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const BUSY_POLL: Duration = Duration::from_millis(5);

/// 실제 타격용이 아닌, 응답 확인용 손목 이동량.
const TEST_STROKE_RAD: f64 = 15.0_f64.to_radians();
const MIN_RAIL_COMMAND_SECS: f64 = 0.05;
const MAX_RAIL_COMMAND_SECS: f64 = 0.30;

#[derive(Default)]
struct TwoStageLatch {
    provisional_sent: bool,
    refined_sent: bool,
}

impl TwoStageLatch {
    fn should_send(&mut self, stage: PredictionStage) -> bool {
        // 2차까지 끝난 뒤 다시 1차가 오면 새 공이다.
        if self.refined_sent && stage == PredictionStage::Provisional {
            *self = Self::default();
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

/// 제어 워커를 띄운다. 실제 장비 동작은 이 워커를 실기 PC에서 실행할 때만 발생한다.
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
            warn!(%error, "초기 센터 이동 실패 — 현재 자세에서 2단계 제어를 시작한다");
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
        let window = motion::InterceptWindow::default();
        let selector =
            HitTargetSelector::new(window.y_min, window.y_max).expect("기본 목표 선택 구간");

        if let Some(sim_tx) = &sim_tx {
            let _ = sim_tx.try_send(SimUpdate {
                pose: Some(PoseMsg::from(&pose)),
                ..SimUpdate::default()
            });
        }
        let _ = event_tx.send(ShotEvent::Armed { shot_seq: 1, pose });
        let _ = status_tx.send(ControlStatus::Ready { shot_seq: 1 });
        info!(
            wrist_ready_rad = f2(ready_wrist),
            "2단계 단순 제어 준비 — 공마다 레일 최대 2회, 손목축만 추가 제어"
        );

        let mut latch = TwoStageLatch::default();
        let mut last_command: Option<Instant> = None;

        while !shutdown.is_down() {
            let request = match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(request) => request,
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            };
            if !latch.should_send(request.stage)
                || request.age_secs() > MAX_REQUEST_AGE_SECS
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
            let rail_x = arm
                .rail
                .map_or(target.position.x, |rail| rail.clamp_x(target.position.x));
            let wrist = test_wrist_goal(ready_wrist, request.stage);
            let duration = remaining.clamp(MIN_RAIL_COMMAND_SECS, MAX_RAIL_COMMAND_SECS);
            if let Err(error) = hardware.command_rail_and_racket(rail_x, wrist, duration) {
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq: 1,
                    reason: format!("레일·라켓 2단계 명령 실패: {error}"),
                });
                break;
            }
            latch.mark_sent(request.stage);
            last_command = Some(Instant::now());

            info!(
                stage = ?request.stage,
                prediction_x = f2(target.position.x),
                prediction_y = f2(target.position.y),
                prediction_z = f2(target.position.z),
                rail_x = f2(rail_x),
                wrist_deg = f2(wrist.to_degrees()),
                remaining_secs = f2(remaining),
                "2단계 단순 제어 명령"
            );
        }

        let _ = event_tx.send(ShotEvent::Done);
    });
}

/// 1차에는 기본각을 유지하고, 2차에만 손목을 15° 전진시킨다.
fn test_wrist_goal(ready: f64, stage: PredictionStage) -> f64 {
    return match stage {
        PredictionStage::Provisional => ready,
        PredictionStage::Refined => ready + TEST_STROKE_RAD,
    };
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

    #[test]
    fn each_prediction_stage_is_sent_only_once_per_ball() {
        let mut latch = TwoStageLatch::default();
        assert!(latch.should_send(PredictionStage::Provisional));
        latch.mark_sent(PredictionStage::Provisional);
        assert!(!latch.should_send(PredictionStage::Provisional));
        assert!(latch.should_send(PredictionStage::Refined));
        latch.mark_sent(PredictionStage::Refined);
        assert!(!latch.should_send(PredictionStage::Refined));

        assert!(latch.should_send(PredictionStage::Provisional));
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
}
