//! sim 카메라 — Rapier 월드의 공 위치를 픽셀로 투영한다.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use pingpong_domain::{CameraId, CameraSource, Clock, FrameRef};
use rapier3d::prelude::Vector;

use super::projection::CameraView;
use super::session::SimClockHandle;
use super::world::SimWorld;

/// Rapier 월드 공을 픽셀로 투영하는 가상 카메라.
pub struct SimCamera {
    /// 이 카메라 ID
    camera_id: CameraId,
    /// 핀홀 투영 설정
    view: CameraView,
    /// 공유 sim 월드
    world: Arc<Mutex<SimWorld>>,
    /// sim 시계
    clock: Arc<SimClockHandle>,
    /// 남은 프레임 (`None` = GUI 무한)
    remaining: Option<u64>,
    /// 프레임 간격
    frame_interval: Duration,
    /// 직전 프레임 시각
    last_frame_at: Option<Instant>,
    /// 종료 신호
    shutdown: Arc<AtomicBool>,
}

impl SimCamera {
    /// `frames == 0` 이면 shutdown까지 무한 프레임.
    pub fn new(
        camera_id: CameraId,
        camera_count: u8,
        frames: u64,
        frame_hz: f64,
        world: Arc<Mutex<SimWorld>>,
        clock: Arc<SimClockHandle>,
        shutdown: Arc<AtomicBool>,
    ) -> Self {
        let remaining = if frames == 0 { None } else { Some(frames) };
        return Self {
            camera_id,
            view: CameraView::for_camera_index(camera_id.index(), camera_count),
            world,
            clock,
            remaining,
            frame_interval: Duration::from_secs_f64(1.0 / frame_hz),
            last_frame_at: None,
            shutdown,
        };
    }
}

impl CameraSource for SimCamera {
    fn next(&mut self) -> Option<(CameraId, FrameRef, Instant)> {
        if self.shutdown.load(Ordering::Acquire) {
            return None;
        }
        if matches!(self.remaining, Some(0)) {
            return None;
        }

        let now = Instant::now();
        if let Some(last) = self.last_frame_at {
            let elapsed = now.duration_since(last);
            if elapsed < self.frame_interval {
                std::thread::sleep(self.frame_interval - elapsed);
            }
        }
        self.last_frame_at = Some(Instant::now());

        if let Some(ref mut n) = self.remaining {
            *n -= 1;
        }
        let timestamp = self.clock.now();

        let ball = self.world.lock().expect("sim 월드").ball_position();
        let frame = project_ball(ball, &self.view);
        return Some((self.camera_id, frame, timestamp));
    }
}

/// Rapier 공 위치 → `FrameRef`.
fn project_ball(ball: Vector, view: &CameraView) -> FrameRef {
    return match view.project(ball) {
        Some(pixel) => FrameRef::sim(pixel),
        None => FrameRef::empty(),
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;
    use std::sync::{Arc, Mutex};

    use crate::sim::controls::SimRuntimeControls;
    use crate::sim::session::SimSessionConfig;
    use crate::sim::SimSession;
    use pingpong_domain::{constants::table, Arm};

    fn test_arm() -> Arc<Arm> {
        return Arc::new(
            Arm::builder()
                .base_xyz(table::WIDTH_X * 0.15, 0.02, table::SURFACE_Z)
                .link(0.35)
                .revolute_at(-1.2, 1.2, 0.0)
                .link(0.30)
                .revolute_at(-0.2, 1.4, 0.6)
                .link(0.15)
                .revolute_at(-1.5, 0.5, -0.4)
                .max_joint_speed(2.5)
                .build()
                .expect("테스트용 3DOF arm"),
        );
    }

    #[test]
    fn sim_camera_emits_frames() {
        let arm = test_arm();
        let controls = Arc::new(Mutex::new(SimRuntimeControls::default()));
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut session = SimSession::new(
            SimSessionConfig {
                physics_hz: 500.0,
                frame_hz: 60.0,
                time_scale: 50.0,
                camera_count: 2,
            },
            arm,
            None,
            controls,
            shutdown,
        );
        let mut camera = session.camera(CameraId::new(0), 3);
        assert!(camera.next().is_some());
        session.shutdown();
    }
}
