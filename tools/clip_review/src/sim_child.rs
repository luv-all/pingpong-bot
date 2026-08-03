//! `--sim-child` — SimScene 자식 창. stdin 한 줄 JSON([`SceneMsg`])으로 먹인다.
//!
//! kiss3d와 OpenCV highgui가 둘 다 메인 스레드를 요구해서 한 프로세스에 못 띄운다
//! (`tools/verify_stereo`와 같은 방식).
//!
//! - **주황 공** = EKF 추정 위치
//! - **반투명 공** = 이 프레임 삼각측량
//! - **초록 선** = 실제 궤적 (지금까지) / 죽인 초록 = 아직 안 온 구간
//! - **회색 선** = 매 프레임 다시 굴린 예측
//! - **자홍 선** = 커밋 순간에 얼린 예측

use std::io::{BufRead, BufReader};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;

use anyhow::Result;
use pingpong_bot::sim::gui::{SimScene, ball, trail};
use pingpong_bot::sim::session::SimRuntimeControls;

use crate::msg::SceneMsg;

// 흰색·하늘색은 밝은 테이블·바닥 위에서 죽는다. 채도 높은 **초록 ↔ 자홍** 보색을 쓴다
// (카메라 창과 같은 규칙).
/// 실제 궤적, 지금까지 — 초록, 굵게.
const OBSERVED_RGBA: [f32; 4] = [0.20, 1.0, 0.20, 1.0];
const OBSERVED_WIDTH: f32 = 4.0;
/// 실제 궤적, 아직 안 온 구간 — 같은 색을 죽여서.
const FUTURE_RGBA: [f32; 4] = [0.10, 0.42, 0.10, 1.0];
const FUTURE_WIDTH: f32 = 2.0;
/// EKF 가 보정한 궤적 — 하늘색. 초록과 나란히 놓여야 필터가 무엇을 폈는지 보인다.
const FILTERED_RGBA: [f32; 4] = [0.0, 0.78, 1.0, 1.0];
const FILTERED_WIDTH: f32 = 3.0;
/// 커밋 순간에 얼린 예측 — 자홍, 굵게. 실제와 벌어진 만큼이 그대로 오차다.
const COMMITTED_RGBA: [f32; 4] = [1.0, 0.15, 1.0, 1.0];
const COMMITTED_WIDTH: f32 = 4.0;

pub fn run() -> Result<()> {
    let shutdown = SimRuntimeControls::new_shutdown();
    let observed = trail::Handle::new(OBSERVED_RGBA, OBSERVED_WIDTH);
    let future = trail::Handle::new(FUTURE_RGBA, FUTURE_WIDTH);
    let filtered = trail::Handle::new(FILTERED_RGBA, FILTERED_WIDTH);
    let committed = trail::Handle::new(COMMITTED_RGBA, COMMITTED_WIDTH);

    let scene = SimScene::builder()
        .title("clip-review sim")
        .with_ball()
        .with_ghost_ball()
        .with_trail(future.clone())
        .with_trail(observed.clone())
        .with_trail(filtered.clone())
        .with_trail(committed.clone())
        .build();
    let ball = scene.ball().expect("with_ball").clone();
    let ghost = scene.ghost_ball_handle().expect("with_ghost_ball").clone();

    let stop = Arc::clone(&shutdown);
    thread::spawn(move || {
        stdin_loop(ball, ghost, observed, future, filtered, committed, stop);
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
    future: trail::Handle,
    filtered: trail::Handle,
    committed: trail::Handle,
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
                future.set_points(SceneMsg::points(&msg.observed_future));
                filtered.set_points(SceneMsg::points(&msg.filtered));
                committed.set_points(SceneMsg::points(&msg.committed));
            }
            Err(error) => eprintln!("sim stdin parse: {error} ({text})"),
        }
    }
}
