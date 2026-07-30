//! 프레임 간격 기반 FPS 추정.

use std::time::Instant;

pub struct FpsMeter {
    last: Option<Instant>,
    pub fps: f64,
}

impl FpsMeter {
    pub fn new() -> Self {
        return Self {
            last: None,
            fps: 0.0,
        };
    }

    pub fn tick(&mut self) {
        let now = Instant::now();
        if let Some(prev) = self.last {
            let dt = now.duration_since(prev).as_secs_f64();
            if dt > 1e-4 {
                let instant = 1.0 / dt;
                self.fps = if self.fps <= 0.0 {
                    instant
                } else {
                    self.fps * 0.85 + instant * 0.15
                };
            }
        }
        self.last = Some(now);
    }
}
