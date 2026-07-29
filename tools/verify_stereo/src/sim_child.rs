//! SimScene 자식 — stdin JSON 한 줄 → BallHandle.

use std::io::{BufRead, BufReader};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;

use anyhow::Result;
use pingpong_bot::{BallHandle, Point3, SimScene, new_shutdown_flag};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct BallMsg {
    x: f64,
    y: f64,
    z: f64,
}

/// `--sim-child` — kiss3d 블로킹. 부모 stdin: `{"x","y","z"}` 또는 `hide`.
pub fn run_sim_child() -> Result<()> {
    let shutdown = new_shutdown_flag();
    let scene = SimScene::builder()
        .title("verify-stereo sim")
        .with_ball()
        .build();
    let ball = scene.ball().expect("with_ball").clone();

    let stop = Arc::clone(&shutdown);
    thread::spawn(move || {
        stdin_loop(ball, stop);
    });

    println!("sim child: reading ball XYZ from stdin");
    scene
        .run(Arc::clone(&shutdown))
        .map_err(anyhow::Error::msg)?;
    shutdown.store(true, Ordering::SeqCst);
    return Ok(());
}

fn stdin_loop(ball: BallHandle, stop: Arc<AtomicBool>) {
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
        if text == "null" || text == "hide" {
            ball.set_position(None);
            continue;
        }
        match serde_json::from_str::<BallMsg>(text) {
            Ok(m) => ball.set_position(Some(Point3::new(m.x, m.y, m.z))),
            Err(e) => eprintln!("sim stdin parse: {e} ({text})"),
        }
    }
}
