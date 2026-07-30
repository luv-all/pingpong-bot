//! 직접 / 스레드 캡처 소스.

use pingpong_bot::camera::{Frame, FrameSource, OpenCvCapture, ThreadedCapture};

pub enum LiveSource {
    Direct(OpenCvCapture),
    Threaded(ThreadedCapture),
}

impl LiveSource {
    pub fn next_frame(&mut self) -> Option<Frame> {
        return match self {
            Self::Direct(c) => c.next_frame(),
            Self::Threaded(c) => c.next_frame(),
        };
    }

    pub fn capture_fps(&self) -> Option<f64> {
        return match self {
            Self::Threaded(c) => Some(c.capture_fps()),
            Self::Direct(_) => None,
        };
    }

    pub fn as_capture_mut(&mut self) -> Option<&mut OpenCvCapture> {
        return match self {
            Self::Direct(c) => Some(c),
            Self::Threaded(_) => None,
        };
    }
}
