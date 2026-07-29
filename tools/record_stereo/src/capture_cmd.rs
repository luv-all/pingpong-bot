//! 캡처 스레드 명령.

use std::path::PathBuf;
use std::time::{Duration, Instant};

pub enum CaptureCmd {
    /// trigger_at = Space 시각. postroll 끝난 뒤 클립 flush.
    Save {
        trigger_at: Instant,
        preroll: Duration,
        postroll: Duration,
        dir: PathBuf,
        scene: String,
        request_fps: f64,
        backend: String,
        fourcc: String,
        width: i32,
        height: i32,
    },
    Stop,
}
