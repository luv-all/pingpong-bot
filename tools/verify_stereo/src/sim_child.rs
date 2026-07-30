//! SimScene 자식 — stdin JSON 한 줄 → sim::gui::ball::Handle.
//!
//! 주황 공 = EKF 출력, 반투명 공 = 생 삼각측량.

use std::io::{BufRead, BufReader};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;

use anyhow::Result;
use pingpong_bot::sim::gui::SimScene;
use pingpong_bot::sim::session::SimRuntimeControls;

use crate::msg::BallMsg;

/// `--sim-child` — kiss3d 블로킹. 부모 stdin: [`BallMsg`] 한 줄 또는 `hide`.
pub fn run_sim_child() -> Result<()> {
    let shutdown = SimRuntimeControls::new_shutdown();
    let scene = SimScene::builder()
        .title("verify-stereo sim")
        .with_ball()
        .with_ghost_ball()
        .build();
    let ball = scene.ball().expect("with_ball").clone();
    let ghost = scene.ghost_ball_handle().expect("with_ghost_ball").clone();

    let stop = Arc::clone(&shutdown);
    thread::spawn(move || {
        stdin_loop(ball, ghost, stop);
    });

    println!("sim child: reading raw/ekf ball XYZ from stdin");
    scene
        .run(Arc::clone(&shutdown))
        .map_err(anyhow::Error::msg)?;
    shutdown.store(true, Ordering::SeqCst);
    return Ok(());
}

fn stdin_loop(
    ball: pingpong_bot::sim::gui::ball::Handle,
    ghost: pingpong_bot::sim::gui::ball::Handle,
    stop: Arc<AtomicBool>,
) {
    let stdin = std::io::stdin();
    let mut lines = BufReader::new(stdin.lock()).lines();
    while !stop.load(Ordering::Relaxed) {
        let Some(line) = lines.next() else {
            break;
        };
        let Ok(line) = line else {
            break;
        };
        let text = line.trim();
        if text.is_empty() {
            continue;
        }
        match BallMsg::parse_line(text) {
            Ok(m) => {
                ball.set_position(m.ekf.map(Into::into));
                ghost.set_position(m.raw.map(Into::into));
            }
            Err(e) => eprintln!("sim stdin parse: {e} ({text})"),
        }
    }
}
