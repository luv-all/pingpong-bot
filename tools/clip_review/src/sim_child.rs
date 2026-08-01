//! `--sim-child` — SimScene 자식 창. stdin 한 줄 JSON([`SceneMsg`])으로 먹인다.
//!
//! kiss3d와 OpenCV highgui가 둘 다 메인 스레드를 요구해서 한 프로세스에 못 띄운다
//! (`tools/verify_stereo`와 같은 방식).
//!
//! - **주황 공** = EKF 추정 위치
//! - **반투명 공** = 이 프레임 삼각측량
//! - **흰 선** = 실제 궤적
//! - **하늘색 선** = 예측 궤적

use std::io::{BufRead, BufReader};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;

use anyhow::Result;
use pingpong_bot::sim::gui::{SimScene, ball, trail};
use pingpong_bot::sim::session::SimRuntimeControls;

use crate::msg::SceneMsg;

/// 실제 궤적 — 흰색, 굵게.
const OBSERVED_RGBA: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
const OBSERVED_WIDTH: f32 = 3.0;
/// 예측 궤적 — 하늘색, 얇게. 카메라 창의 하늘색과 같은 뜻.
const PREDICTED_RGBA: [f32; 4] = [0.35, 0.95, 1.0, 1.0];
const PREDICTED_WIDTH: f32 = 2.0;

pub fn run() -> Result<()> {
    let shutdown = SimRuntimeControls::new_shutdown();
    let observed = trail::Handle::new(OBSERVED_RGBA, OBSERVED_WIDTH);
    let predicted = trail::Handle::new(PREDICTED_RGBA, PREDICTED_WIDTH);

    let scene = SimScene::builder()
        .title("clip-review sim")
        .with_ball()
        .with_ghost_ball()
        .with_trail(observed.clone())
        .with_trail(predicted.clone())
        .build();
    let ball = scene.ball().expect("with_ball").clone();
    let ghost = scene.ghost_ball_handle().expect("with_ghost_ball").clone();

    let stop = Arc::clone(&shutdown);
    thread::spawn(move || {
        stdin_loop(ball, ghost, observed, predicted, stop);
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
    observed: trail::Handle,
    predicted: trail::Handle,
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
        match SceneMsg::parse_line(text) {
            Ok(msg) => {
                ball.set_position(msg.ekf.map(Into::into));
                ghost.set_position(msg.raw.map(Into::into));
                observed.set_points(SceneMsg::points(&msg.observed));
                predicted.set_points(SceneMsg::points(&msg.predicted));
            }
            Err(error) => eprintln!("sim stdin parse: {error} ({text})"),
        }
    }
}
