//! sim용 가짜 카메라.

use std::time::{Duration, Instant};

use pingpong_domain::{CameraSource, CameraId, Clock, FrameRef, PixelPoint};

use crate::clock::SystemClock;

/// sin/cos로 움직이는 픽셀을 생성하는 레거시 가짜 카메라.
pub struct SyntheticCamera {
    /// 카메라 ID
    camera_id: CameraId,
    /// OS 시계
    clock: SystemClock,
    /// 프레임 간격 (~120Hz)
    frame_interval: Duration,
    /// 남은 프레임 수
    remaining: u64,
    /// 픽셀 x 기준값
    base_x: f64,
    /// 픽셀 y 기준값
    base_y: f64,
    /// 직전 프레임 시각
    last_tick: Option<Instant>,
}

impl SyntheticCamera {
    /// ID·프레임 수로 가짜 카메라를 만든다.
    pub fn new(camera_id: CameraId, frames: u64) -> Self {
        let index = f64::from(camera_id.index());
        let base_x = 320.0 + index * 320.0;
        let base_y = 240.0;

        return Self {
            camera_id,
            clock: SystemClock,
            frame_interval: Duration::from_micros(8333),
            remaining: frames,
            base_x,
            base_y,
            last_tick: None,
        };
    }
}

impl CameraSource for SyntheticCamera {
    fn next(&mut self) -> Option<(CameraId, FrameRef, Instant)> {
        if self.remaining == 0 {
            return None;
        }

        let now = self.clock.now();
        if let Some(last) = self.last_tick {
            let elapsed = now.duration_since(last);
            if elapsed < self.frame_interval {
                std::thread::sleep(self.frame_interval - elapsed);
            }
        }
        let timestamp = self.clock.now();
        self.last_tick = Some(timestamp);

        self.remaining -= 1;
        let phase = self.remaining as f64 * 0.05;
        let pixel = PixelPoint::new(
            self.base_x + phase.sin() * 20.0,
            self.base_y + phase.cos() * 10.0,
        );

        return Some((self.camera_id, FrameRef::sim(pixel), timestamp));
    }
}
