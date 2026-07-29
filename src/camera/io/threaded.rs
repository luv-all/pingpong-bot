//! 백그라운드 grab + 최신 프레임 (hinguri Camera::update 알맹이).

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use opencv::prelude::*;

use super::capture::{Frame, FrameSource, OpenCvCapture};
use crate::Id;

/// 첫 프레임 대기 상한 (MSMF 콜드스타트용).
const FIRST_FRAME_WAIT: Duration = Duration::from_secs(8);
const POLL_INTERVAL: Duration = Duration::from_millis(5);

struct LatestSlot {
    image: opencv::core::Mat,
    timestamp: Instant,
    seq: u64,
}

/// `OpenCvCapture`를 전용 스레드에서 돌리고, 소비자는 최신 프레임만 읽는다.
///
/// UI/검출이 느려도 캡처는 계속 진행되어 USB 버퍼가 쌓이지 않는다.
pub struct ThreadedCapture {
    camera_id: Id,
    latest: Arc<Mutex<Option<LatestSlot>>>,
    stop: Arc<AtomicBool>,
    /// grab 루프가 `read` 실패로 종료됨.
    grab_ended: Arc<AtomicBool>,
    grab_count: Arc<AtomicU64>,
    join: Option<JoinHandle<()>>,
    #[allow(dead_code)]
    last_served_seq: u64,
    capture_fps: f64,
    fps_window_start: Instant,
    fps_window_count: u64,
}

impl ThreadedCapture {
    pub fn spawn(mut inner: OpenCvCapture) -> Self {
        let camera_id = inner.camera_id();
        let latest: Arc<Mutex<Option<LatestSlot>>> = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let grab_ended = Arc::new(AtomicBool::new(false));
        let grab_count = Arc::new(AtomicU64::new(0));
        let latest_t = Arc::clone(&latest);
        let stop_t = Arc::clone(&stop);
        let ended_t = Arc::clone(&grab_ended);
        let grab_t = Arc::clone(&grab_count);
        let join = thread::spawn(move || {
            while !stop_t.load(Ordering::Relaxed) {
                let Some(frame) = inner.next_frame() else {
                    break;
                };
                grab_t.fetch_add(1, Ordering::Relaxed);
                if let Ok(mut slot) = latest_t.lock() {
                    let seq = match slot.as_ref() {
                        Some(s) => s.seq + 1,
                        None => 1,
                    };
                    *slot = Some(LatestSlot {
                        image: frame.image,
                        timestamp: frame.timestamp,
                        seq,
                    });
                }
            }
            ended_t.store(true, Ordering::Release);
        });
        return Self {
            camera_id,
            latest,
            stop,
            grab_ended,
            grab_count,
            join: Some(join),
            last_served_seq: 0,
            capture_fps: 0.0,
            fps_window_start: Instant::now(),
            fps_window_count: 0,
        };
    }

    pub fn camera_id(&self) -> Id {
        return self.camera_id;
    }

    /// grab 스레드가 측정한 대략적 캡처 FPS (최근 1초 창).
    pub fn capture_fps(&self) -> f64 {
        return self.capture_fps;
    }

    fn refresh_capture_fps(&mut self) {
        let total = self.grab_count.load(Ordering::Relaxed);
        let elapsed = self.fps_window_start.elapsed().as_secs_f64();
        if elapsed >= 1.0 {
            let delta = total.saturating_sub(self.fps_window_count);
            self.capture_fps = delta as f64 / elapsed;
            self.fps_window_count = total;
            self.fps_window_start = Instant::now();
        }
    }

    fn try_clone_latest(&self) -> Option<Frame> {
        let guard = self.latest.lock().ok()?;
        let slot = guard.as_ref()?;
        let image = slot.image.try_clone().ok()?;
        return Some(Frame::new(self.camera_id, image, slot.timestamp));
    }
}

impl FrameSource for ThreadedCapture {
    fn next_frame(&mut self) -> Option<Frame> {
        self.refresh_capture_fps();
        if let Some(frame) = self.try_clone_latest() {
            return Some(frame);
        }
        // 스폰 직후·MSMF 콜드스타트: 첫 프레임이 올 때까지 짧게 폴링.
        // (없으면 cam-preview가 즉시 "프레임 끝/실패"로 죽음)
        let deadline = Instant::now() + FIRST_FRAME_WAIT;
        while Instant::now() < deadline {
            if self.grab_ended.load(Ordering::Acquire) {
                return None;
            }
            thread::sleep(POLL_INTERVAL);
            if let Some(frame) = self.try_clone_latest() {
                return Some(frame);
            }
        }
        return None;
    }

    fn camera_id(&self) -> Id {
        return self.camera_id;
    }
}

impl Drop for ThreadedCapture {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}
