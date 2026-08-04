//! 실물 고정 스윙 딕셔너리 제어 워커 — IK 없음.
//!
//! `control_worker.rs`(2단계 `PositionController`, 5차원/3차원 IK)와 별개의
//! 워커다. 이 경로는 `HitTargetSelector::select`의 기하 보간(궤적 예측 행
//! 사이 선형보간)만 써서 레일 x·타이밍을 정하고, 관절은 항상 고정 딕셔너리
//! (`robot::motion::{fixed_swing_start_joints, fixed_swing_end_joints}`)를
//! 그대로 재생한다.

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender};
use pingpong_bot::error::HwError;
use pingpong_bot::estimator::BallTrajectory;
use pingpong_bot::hardware::Hardware;
use pingpong_bot::robot::Arm;
use pingpong_bot::robot::control::{HitTarget, HitTargetSelector, TargetSelectionError};
use pingpong_bot::robot::motion::{InterceptWindow, Planner};
use tracing::{info, info_span, warn};

use super::{CommitRequest, ControlStatus, PoseMsg, ShotEvent, Shutdown, SimUpdate, SwingMsg};

const MAX_REQUEST_AGE_SECS: f64 = 0.250;
const COMMAND_THROTTLE: Duration = Duration::from_millis(20);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);

/// `trajectory.predicted`에서 `window`의 중앙 y를 기하 보간만으로 읽는다 —
/// IK 없음. `HitTargetSelector::select`가 이미 하는 일을 그대로 노출해
/// 단위테스트를 IK/하드웨어 없이 돌릴 수 있게 한 얇은 래퍼.
fn target_from_ball_trajectory(
    trajectory: &BallTrajectory,
    window: InterceptWindow,
) -> Result<HitTarget, TargetSelectionError> {
    let selector = HitTargetSelector::new(window.y_min, window.y_max)
        .map_err(|_| TargetSelectionError::InvalidWindow)?;
    return selector.select(trajectory);
}

/// 제어 워커를 띄운다. `control_worker::spawn`과 같은 시그니처 — `real/run.rs`가
/// 둘 중 하나를 고른다.
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
        let _span = info_span!("fixed_swing_control").entered();

        if home && let Err(error) = move_to_start(hardware.as_mut(), &arm) {
            warn!(%error, "초기 스윙 시작 자세 정렬 실패 — 2단계 제어를 시작하지 않는다");
            let _ = event_tx.send(ShotEvent::Failed {
                shot_seq: 1,
                reason: format!("초기 정렬 실패: {error}"),
            });
            let _ = event_tx.send(ShotEvent::Done);
            return;
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
        if let Some(sim_tx) = &sim_tx {
            let _ = sim_tx.try_send(SimUpdate {
                pose: Some(PoseMsg::from(&pose)),
                ..SimUpdate::default()
            });
        }
        let mut shot_seq: u64 = 1;
        let _ = event_tx.send(ShotEvent::Armed { shot_seq, pose });
        let _ = status_tx.send(ControlStatus::Ready { shot_seq });
        info!("고정 스윙 딕셔너리 제어 준비 — IK 없음");

        let mut last_command: Option<Instant> = None;
        let mut committed_this_ball = false;

        while !shutdown.is_down() {
            let request = match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(request) => request,
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => continue,
            };
            if committed_this_ball
                || request.age_secs() > MAX_REQUEST_AGE_SECS
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
                || hardware.is_busy()
            {
                continue;
            }

            let Ok(target) = target_from_ball_trajectory(&request.trajectory, intercept) else {
                continue;
            };
            let Some(rail) = arm.rail else {
                continue;
            };
            let rail_x = pingpong_bot::robot::motion::fixed_swing_rail_target(&rail, target.position.x);

            let band =
                pingpong_bot::robot::motion::SwingHeightBand::for_impact_z(target.position.z);
            let Ok(trajectory) = Planner::plan_fixed_swing(
                &arm,
                rail_x,
                pingpong_bot::robot::motion::DEFAULT_SWING_SHAPE_STRATEGY,
                band,
            ) else {
                continue;
            };
            let remaining_secs =
                target.time_secs - request.trajectory.reference_time.elapsed().as_secs_f64();
            // Task 3b: 스윙 전체 소요 시간이 아니라 스윙 내부 임팩트 시각을
            // 기준으로 삼는다 — 라켓은 START→END를 스윕하는 도중 공을 만나야
            // 하고, 스윙이 "끝나는" 시점을 임팩트로 보면 안 된다.
            let impact_time = Planner::fixed_swing_impact_time_secs(
                &arm,
                rail_x,
                &trajectory,
                pingpong_bot::robot::motion::DEFAULT_IMPACT_TIME_STRATEGY,
            );
            if !pingpong_bot::robot::motion::should_start_fixed_swing(remaining_secs, impact_time)
            {
                continue;
            }

            if let Err(error) = hardware.command(&trajectory) {
                let _ = event_tx.send(ShotEvent::Failed {
                    shot_seq,
                    reason: format!("고정 스윙 명령 실패: {error}"),
                });
                break;
            }
            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    swing: Some(SwingMsg::from_trajectory(&trajectory)),
                    ..SimUpdate::default()
                });
            }
            committed_this_ball = true;
            last_command = Some(Instant::now());
            let _ = event_tx.send(ShotEvent::Committed {
                shot_seq,
                time_to_impact_secs: remaining_secs.max(0.0),
                duration_secs: trajectory.duration_secs,
                impact: target.position,
                rail_start: trajectory.rail.start,
                rail_end: trajectory.rail.end,
                peak_joint_speed: trajectory.peak_joint_speed(),
            });
            info!(
                shot = shot_seq,
                rail_x,
                duration_secs = trajectory.duration_secs,
                remaining_secs,
                "real shot: fixed swing dictionary commit (no IK)"
            );

            // 재생 완료를 기다린 뒤 시작 자세로 복귀하고 다음 공을 받는다.
            while hardware.is_busy() {
                thread::sleep(Duration::from_millis(5));
            }
            if let Err(error) = move_to_start(hardware.as_mut(), &arm) {
                warn!(%error, "스윙 시작 자세 복귀 실패 — 현재 자세에서 계속");
            }
            let _ = status_tx.send(ControlStatus::Recovering { shot_seq });
            shot_seq = shot_seq.saturating_add(1);
            committed_this_ball = false;
            let _ = status_tx.send(ControlStatus::Ready { shot_seq });
        }

        let _ = event_tx.send(ShotEvent::Done);
    });
}

/// 레일 중앙 + 고정 딕셔너리 시작 자세로 이동한다 — `control_worker::move_to_center`와
/// 같은 자리지만 목표가 `arm.default_joints`가 아니라 스윙 시작 딕셔너리다.
fn move_to_start(hardware: &mut dyn Hardware, arm: &Arm) -> Result<(), HwError> {
    let rail_center = arm.rail.map_or(0.0, |rail| rail.default_x());
    return hardware.command_initial_pose(rail_center, &Planner::fixed_swing_start_joints());
}

#[cfg(test)]
mod tests {
    use pingpong_bot::Point3;
    use pingpong_bot::estimator::{BallTrajectory, TrajectorySample};
    use pingpong_bot::robot::motion::InterceptWindow;

    use super::target_from_ball_trajectory;

    #[test]
    fn target_from_ball_trajectory_reads_x_and_time_with_no_ik() {
        let window = InterceptWindow {
            y_min: 0.2,
            y_max: 0.4,
            sample_step: 0.03,
        };
        let trajectory = BallTrajectory::new(
            vec![],
            vec![
                TrajectorySample::new(
                    Point3::new(0.2, 0.2, 0.4),
                    nalgebra::Vector3::new(0.0, -2.0, 0.0),
                    0.10,
                ),
                TrajectorySample::new(
                    Point3::new(0.4, 0.4, 0.2),
                    nalgebra::Vector3::new(0.0, -2.0, 0.0),
                    0.20,
                ),
            ],
            std::time::Instant::now(),
        )
        .expect("valid trajectory");

        let target = target_from_ball_trajectory(&trajectory, window).expect("target");
        assert!((target.position.x - 0.3).abs() < 1e-9);
        assert!((target.time_secs - 0.15).abs() < 1e-9);
    }
}
