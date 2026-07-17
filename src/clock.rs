//! 시계 어댑터.
//!
//! - `SystemClock`: 실시간 (실물·sim 공통)
//! - `SimClock`: 시간 가속 — Rapier sim에서 재현 가능한 타임라인용 (2단계)

use std::time::{Duration, Instant};

/// monotonic 시각. sim에서는 시간 가속이 가능하다.
pub trait Clock: Send {
    fn now(&self) -> Instant;
}

/// OS 시스템 시각을 그대로 사용한다.
pub struct SystemClock;

impl Clock for SystemClock {
    fn now(&self) -> Instant {
        return Instant::now();
    }
}

/// sim 전용 — 경과 시간에 scale을 곱해 빠르게 돌릴 수 있다.
pub struct SimClock {
    /// 기준 시각
    origin: Instant,
    /// 시간 배율 (1.0 = 실시간)
    scale: f64,
}

impl SimClock {
    /// 배율을 지정해 sim 시계를 만든다.
    pub fn new(scale: f64) -> Self {
        return Self {
            origin: Instant::now(),
            scale,
        };
    }
}

impl Clock for SimClock {
    fn now(&self) -> Instant {
        return self.origin
            + Duration::from_secs_f64(self.origin.elapsed().as_secs_f64() * self.scale);
    }
}
