//! 녹화 클립을 **실시간 속도로** 재생하는 `FrameSource` 래퍼.
//!
//! `OpenCvCapture::from_path` + `set_timeline_fps`는 프레임 타임스탬프를
//! `epoch + index/fps`로 합성한다 (`epoch`는 클립을 연 시각). 그래서 타임스탬프는 **미래의
//! 실제 `Instant`** 이고, 그 시각까지 기다리면 녹화 당시 속도로 재생된다.
//!
//! 페이싱이 없으면 디코드가 되는 대로 프레임이 쏟아져, 벽시계로 도는 것들(계획 스로틀 20 ms,
//! 제어 요청 신선도와 하드웨어 executor의 `stream_hz`)이 클립 시간과 어긋난다 —
//! 라이브와 다른 코드 경로를 시험하게 된다.

use std::thread;
use std::time::Instant;

use pingpong_bot::camera;
use pingpong_bot::camera::{Frame, FrameSource};

/// 프레임 타임스탬프가 될 때까지 기다렸다 내보낸다.
pub struct PacedSource {
    inner: Box<dyn FrameSource>,
}

impl PacedSource {
    pub fn new(inner: Box<dyn FrameSource>) -> Self {
        return Self { inner };
    }
}

impl FrameSource for PacedSource {
    fn next_frame(&mut self) -> Option<Frame> {
        let frame = self.inner.next_frame()?;
        // 합성 타임스탬프가 아직 미래면 그때까지 잔다. 이미 지났으면 (디코드가 느리면)
        // 그냥 통과시킨다 — 밀린 프레임을 몰아치기로 내보내지 않는다.
        let now = Instant::now();
        if frame.timestamp > now {
            thread::sleep(frame.timestamp - now);
        }
        return Some(frame);
    }

    fn camera_id(&self) -> camera::Id {
        return self.inner.camera_id();
    }
}
