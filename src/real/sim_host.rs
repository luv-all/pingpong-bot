//! 부모 쪽 sim 자식 프로세스 관리 — 띄우고, 먹이고, 거둔다.

use std::io::Write;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::thread::{self, JoinHandle};

use crossbeam_channel::{Receiver, RecvTimeoutError};
use pingpong_bot::robot::RailFrame;
use tracing::{info, warn};

use super::{SIM_CHILD_FLAG, SimUpdate};

const RECV_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(100);

/// sim 자식을 띄우고, 채널로 오는 갱신을 stdin으로 흘려보내는 스레드를 건다.
///
/// 자식 띄우기에 실패해도 **본 파이프라인은 계속 간다** — 관전용 창일 뿐이다.
pub fn spawn(rx: Receiver<SimUpdate>, rail_frame: RailFrame) -> Option<JoinHandle<()>> {
    let (mut child, mut stdin) = match start(rail_frame) {
        Ok(pair) => pair,
        Err(error) => {
            warn!(%error, "sim 창 띄우기 실패 — 프리뷰·로그만으로 진행");
            return None;
        }
    };
    info!("sim 관전 창 (자식 프로세스)");

    return Some(thread::spawn(move || {
        loop {
            match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(update) => {
                    if writeln!(stdin, "{}", update.to_line()).is_err() || stdin.flush().is_err() {
                        // 사용자가 sim 창을 닫았다 — 조용히 그만 보낸다.
                        break;
                    }
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        drop(stdin);
        let _ = child.wait();
    }));
}

fn start(rail_frame: RailFrame) -> Result<(Child, ChildStdin), String> {
    let exe = std::env::current_exe().map_err(|error| format!("current_exe: {error}"))?;
    let mut child = Command::new(exe)
        .arg(SIM_CHILD_FLAG)
        .arg(rail_frame.table_distance_m().to_string())
        .arg(rail_frame.rail_bottom_z.to_string())
        .stdin(Stdio::piped())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|error| format!("spawn: {error}"))?;
    let stdin = child
        .stdin
        .take()
        .ok_or_else(|| "sim 자식 stdin 없음".to_owned())?;
    return Ok((child, stdin));
}
