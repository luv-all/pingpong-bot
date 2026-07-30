//! `--sim-child` — 조작 없는 관전용 kiss3d 창.
//!
//! kiss3d와 OpenCV highgui가 둘 다 메인 스레드를 요구해서 한 프로세스에 못 띄운다.
//! `tools/verify_stereo`와 같이 부모가 자기 자신을 이 플래그로 띄우고 stdin 한 줄 JSON
//! ([`SimUpdate`])으로 먹인다. 이 창은 **아무것도 조작하지 않는다** — 실기가 무엇을 보고
//! 무엇을 하려는지 그대로 비춰 보기만 한다.
//!
//! - 주황 공 = EKF 추정 공 위치
//! - 반투명 공 = 예측 도달 위치 (접수 평면 교차)
//! - 로봇 = 실기에서 읽은 포즈. 커밋되면 그 궤적을 재생한다

use std::io::{BufRead, BufReader};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::{Context, Result};
use pingpong_bot::defaults::robot;
use pingpong_bot::sim::gui::{SimScene, ball, robot as gui_robot};
use pingpong_bot::sim::physics::world::SimWorld;
use pingpong_bot::sim::session::SimRuntimeControls;
use tracing::warn;

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
        .ghost_ball(true)
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
    while !stop.load(Ordering::Relaxed) {
        let Some(Ok(line)) = lines.next() else {
            break;
        };
        let text = line.trim();
        if text.is_empty() {
            continue;
        }
        match SimUpdate::parse_line(text) {
            Ok(update) => apply(&ball, &ghost, &robot, update),
            Err(error) => warn!(%error, text, "sim stdin 파싱 실패"),
        }
    }
}

fn apply(ball: &ball::Handle, ghost: &ball::Handle, robot: &gui_robot::Handle, update: SimUpdate) {
    ball.set_position(update.ball);
    ghost.set_position(update.impact);
    // 스윙 재생 중에는 포즈를 덮어쓰지 않는다 — 재생이 끊긴다.
    if let Some(swing) = &update.swing {
        robot.play(swing.to_trajectory());
    } else if let Some(pose) = &update.pose
        && !robot.is_busy()
    {
        robot.set_pose(pose.into());
    }
}
