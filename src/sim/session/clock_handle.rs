//! sim 경과 시간을 `Instant`로 노출하는 시계.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// sim 경과 시간을 `Instant`로 노출하는 시계.
pub struct SimClockHandle {
    /// wall-clock 기준 원점
    origin: Instant,
    /// 공유 sim 시간 [s]
    sim_time: Arc<Mutex<f64>>,
}

impl SimClockHandle {
    /// sim 시간 뮤텍스로 핸들을 만든다.
    pub(crate) fn new(sim_time: Arc<Mutex<f64>>) -> Self {
        return Self {
            origin: Instant::now(),
            sim_time,
        };
    }

    /// 현재 sim time [s].
    pub fn sim_time_secs(&self) -> f64 {
        return *self.sim_time.lock().expect("sim 시간");
    }

    /// sim 경과를 wall `Instant`로 노출 (관측 타임스탬프용).
    pub fn now(&self) -> Instant {
        let secs = *self.sim_time.lock().expect("sim 시간");
        return self.origin + Duration::from_secs_f64(secs);
    }
}
