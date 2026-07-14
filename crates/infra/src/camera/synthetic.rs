//! sim용 가짜 카메라.

use std::time::{Duration, Instant};

use pingpong_domain::{CameraId, CameraSource, Clock, PixelPoint};

use crate::clock::SystemClock;

/// sin/cos 로 움직이는 픽셀을 만드는 레거시 가짜 카메라.
pub struct SyntheticCamera {
    camera_id: CameraId,
    clock: SystemClock,
    frame_interval: Duration,
    remaining: u64,
    base_x: f64,
    base_y: f64,
    last_tick: Option<Instant>,
}

impl SyntheticCamera {
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
    fn next(&mut self) -> Option<(CameraId, Option<PixelPoint>, Instant)> {
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

        return Some((self.camera_id, Some(pixel), timestamp));
    }
}
