//! `--sim-child` — 조작 없는 관전용 kiss3d 창.
//!
//! kiss3d와 OpenCV highgui가 둘 다 메인 스레드를 요구해서 한 프로세스에 못 띄운다.
//! `tools/verify_stereo`와 같이 부모가 자기 자신을 이 플래그로 띄우고 stdin 한 줄 JSON
//! ([`SimUpdate`])으로 먹인다. 이 창은 **아무것도 조작하지 않는다** — 실기가 무엇을 보고
//! 무엇을 하려는지 그대로 비춰 보기만 한다.
//!
//! - **주황** 공 = 새 `vision::Fit` 추정 공 위치
//! - **하늘색** 공 = 현재 선택한 제어 목표 위치
//! - 로봇 = 실기에서 읽은 시작 포즈
//!
//! 구분은 **색**이다. `ball::Visual::spawn_ghost`가 알파 0.38을 주지만 렌더러가 알파를
//! 블렌딩하지 않아 둘 다 불투명하게 보인다 — "반투명 고스트"로 읽지 말 것.

use std::io::{BufRead, BufReader};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::{Context, Result};
use pingpong_bot::defaults::robot;
use pingpong_bot::sim::gui::{SimScene, ball, robot as gui_robot};
use pingpong_bot::sim::physics::world::SimWorld;
use pingpong_bot::sim::session::SimRuntimeControls;
use tracing::{info, warn};

use super::SimUpdate;

/// kiss3d 블로킹 실행. 부모가 죽으면 stdin EOF로 같이 내려간다.
pub fn run() -> Result<()> {
    let shutdown = SimRuntimeControls::new_shutdown();
    let robot = robot().context("defaults::robot")?;

    // 물리를 돌리지 않는 관전용 월드 — jog와 같은 kinematic 설정.
    let world = Arc::new(Mutex::new(SimWorld::new(robot.clone())));
    {
        let mut world = world.lock().expect("sim 월드");
        world.set_use_ground_truth(false);
        world.set_kinematic_robot(true);
        world.robot_mut().set_auto_return_to_center(false);
    }

    let scene = SimScene::builder()
        .title("real shot sim (관전 전용)")
        .with_robot(Arc::clone(&world))
        .with_ball()
        .with_ghost_ball()
        // `ghost_ball(true)`는 **본 공까지** ghost 색으로 만든다 — 그러면 Fit 공과 도달점이
        // 똑같은 하늘색이 돼 구분이 안 된다. 본 공은 주황(`spawn`) 그대로 둔다.
        .urdf(robot.urdf.clone())
        .build();

    let ball_handle = scene.ball().context("with_ball").cloned()?;
    let ghost = scene
        .ghost_ball_handle()
        .context("with_ghost_ball")
        .cloned()?;
    let robot_handle = scene.robot().context("with_robot").cloned()?;

    let stop = Arc::clone(&shutdown);
    thread::spawn(move || {
        stdin_loop(ball_handle, ghost, robot_handle, stop);
    });

    scene
        .run(Arc::clone(&shutdown))
        .map_err(anyhow::Error::msg)?;
    shutdown.store(true, Ordering::SeqCst);
    return Ok(());
}

fn stdin_loop(
    ball: ball::Handle,
    ghost: ball::Handle,
    robot: gui_robot::Handle,
    stop: Arc<AtomicBool>,
) {
    let stdin = std::io::stdin();
    let mut lines = BufReader::new(stdin.lock()).lines();
    let (mut received, mut with_target) = (0_u64, 0_u64);
    let mut last_ball_present: Option<bool> = None;

    while !stop.load(Ordering::Relaxed) {
        let Some(Ok(line)) = lines.next() else {
            break;
        };
        let text = line.trim();
        if text.is_empty() {
            continue;
        }
        match SimUpdate::parse_line(text) {
            Ok(update) => {
                received += 1;
                if update.target.is_some() {
                    with_target += 1;
                }
                if let Some(ball_present) = ball_transition(&mut last_ball_present, &update) {
                    info!(
                        received,
                        with_target,
                        ball = ball_present,
                        "sim: 공 감지 상태 변경"
                    );
                }
                apply(&ball, &ghost, &robot, update);
            }
            Err(error) => warn!(%error, text, "sim stdin 파싱 실패"),
        }
    }
    info!(received, with_target, "sim: stdin 종료");
}

/// `pose` 전용 패킷의 `ball=None`은 "공 없음"이 아니라 "공 필드 미전송"이다.
/// 추정 패킷에서 감지 상태가 실제로 바뀐 경우만 새 값을 반환한다.
fn ball_transition(last: &mut Option<bool>, update: &SimUpdate) -> Option<bool> {
    if update.pose.is_some() {
        return None;
    }
    let current = update.ball.is_some();
    if *last == Some(current) {
        return None;
    }
    *last = Some(current);
    return Some(current);
}

/// 준 필드만 반영한다.
///
/// 제어 워커는 pose를 보내고 추정 워커는 ball·target을 보낸다 —
/// 없는 필드를 `None`으로 덮어쓰면 서로가 서로를 지운다.
fn apply(ball: &ball::Handle, ghost: &ball::Handle, robot: &gui_robot::Handle, update: SimUpdate) {
    if update.ball.is_some() {
        ball.set_position(update.ball);
    }
    if update.target.is_some() {
        ghost.set_position(update.target);
    }
    if let Some(pose) = &update.pose {
        robot.set_pose(pose.into());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::real::PoseMsg;
    use pingpong_bot::Point3;

    #[test]
    fn ball_log_is_emitted_only_when_detection_state_changes() {
        let mut last = None;
        let no_ball = SimUpdate::default();
        assert_eq!(ball_transition(&mut last, &no_ball), Some(false));
        assert_eq!(ball_transition(&mut last, &no_ball), None);

        let ball = SimUpdate {
            ball: Some(Point3::new(0.1, 0.2, 0.3)),
            ..SimUpdate::default()
        };
        assert_eq!(ball_transition(&mut last, &ball), Some(true));
        assert_eq!(ball_transition(&mut last, &ball), None);

        let pose_only = SimUpdate {
            pose: Some(PoseMsg {
                rail_x: 0.0,
                joints: vec![0.0; 4],
            }),
            ..SimUpdate::default()
        };
        assert_eq!(ball_transition(&mut last, &pose_only), None);
        assert_eq!(last, Some(true));
    }
}
