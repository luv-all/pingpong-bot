//! sim 세션 — 물리 스레드와 공유 월드.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::Clock;
use tracing::info;

use super::controls::SimRuntimeControls;
use super::world::{SimStepInput, SimWorld};
use crate::camera::SimCamera;
use crate::hardware::SimHardware;
use crate::robot::urdf::UrdfRobot;

/// sim 실행 설정.
#[derive(Debug, Clone, Copy)]
pub struct SimSessionConfig {
    /// 물리 적분 주파수 [Hz] — 공 CCD용 (plan §9)
    pub physics_hz: f64,
    /// 가상 카메라 프레임률 [Hz]
    pub frame_hz: f64,
    /// 1.0 = 실시간, 10.0 = 10배속
    pub time_scale: f64,
    /// sim 가상 카메라 대수
    pub camera_count: u8,
}

impl Default for SimSessionConfig {
    fn default() -> Self {
        return Self {
            physics_hz: 1000.0,
            frame_hz: 120.0,
            time_scale: 1.0,
            camera_count: 3,
        };
    }
}

/// sim 경과 시간을 `Instant`로 노출하는 시계.
pub struct SimClockHandle {
    /// wall-clock 기준 원점
    origin: Instant,
    /// 공유 sim 시간 [s]
    sim_time: Arc<Mutex<f64>>,
}

impl SimClockHandle {
    /// sim 시간 뮤텍스로 핸들을 만든다.
    fn new(sim_time: Arc<Mutex<f64>>) -> Self {
        return Self {
            origin: Instant::now(),
            sim_time,
        };
    }

    /// 현재 sim time [s].
    pub fn sim_time_secs(&self) -> f64 {
        return *self.sim_time.lock().expect("sim 시간");
    }
}

impl Clock for SimClockHandle {
    fn now(&self) -> Instant {
        let secs = *self.sim_time.lock().expect("sim 시간");
        return self.origin + Duration::from_secs_f64(secs);
    }
}

/// Rapier 디지털 트윈 세션.
pub struct SimSession {
    /// 공유 물리 월드
    world: Arc<Mutex<SimWorld>>,
    /// sim 시계
    clock: Arc<SimClockHandle>,
    /// 종료 플래그
    shutdown: Arc<AtomicBool>,
    /// 물리 적분 스레드
    physics_handle: Option<JoinHandle<()>>,
    /// 세션 설정
    config: SimSessionConfig,
    /// GUI·발사 제어
    controls: Arc<Mutex<SimRuntimeControls>>,
}

impl SimSession {
    /// `arm` — sim·real·제어가 공유하는 불변 로봇 모델 (plan §2).
    pub fn new(
        config: SimSessionConfig,
        arm: Arc<crate::Arm>,
        urdf: Option<Arc<UrdfRobot>>,
        controls: Arc<Mutex<SimRuntimeControls>>,
        shutdown: Arc<AtomicBool>,
    ) -> Self {
        return Self::with_physics(
            config,
            arm,
            urdf,
            controls,
            shutdown,
            crate::PhysicsParams::default(),
        );
    }

    /// config `[physics]`를 Rapier 월드에 반영한다.
    pub fn with_physics(
        config: SimSessionConfig,
        arm: Arc<crate::Arm>,
        urdf: Option<Arc<UrdfRobot>>,
        controls: Arc<Mutex<SimRuntimeControls>>,
        shutdown: Arc<AtomicBool>,
        physics: crate::PhysicsParams,
    ) -> Self {
        let world = Arc::new(Mutex::new(SimWorld::with_physics(arm, urdf, physics)));
        let sim_time = Arc::new(Mutex::new(0.0_f64));
        let clock = Arc::new(SimClockHandle::new(Arc::clone(&sim_time)));
        let physics_shutdown = Arc::clone(&shutdown);

        {
            let mut ctrl = controls.lock().expect("sim controls");
            ctrl.time_scale = config.time_scale;
        }

        let physics_world = Arc::clone(&world);
        let physics_controls = Arc::clone(&controls);
        let physics_time = Arc::clone(&sim_time);
        let physics_dt = 1.0 / config.physics_hz;

        let physics_handle = thread::spawn(move || {
            let wall_origin = Instant::now();
            loop {
                if physics_shutdown.load(Ordering::Acquire) {
                    break;
                }

                let time_scale = {
                    physics_controls
                        .lock()
                        .expect("sim controls")
                        .time_scale
                        .max(0.01)
                };

                let target_sim = wall_origin.elapsed().as_secs_f64() * time_scale;
                let mut catchup_steps = 0_u32;
                const MAX_CATCHUP_STEPS: u32 = 8;
                loop {
                    if physics_shutdown.load(Ordering::Acquire) {
                        return;
                    }
                    let current = {
                        let w = physics_world.lock().expect("sim 월드");
                        w.sim_time
                    };
                    if current >= target_sim {
                        break;
                    }
                    let (shoot, park, shooter, ball_script) = {
                        let mut ctrl = physics_controls.lock().expect("sim controls");
                        let shoot = ctrl.shoot_requested;
                        let park = ctrl.park_requested;
                        ctrl.shoot_requested = false;
                        ctrl.park_requested = false;
                        let script = std::mem::take(&mut ctrl.ball_script_queue);
                        (shoot, park, ctrl.shooter.clone(), script)
                    };
                    let mut w = physics_world.lock().expect("sim 월드");
                    if !ball_script.is_empty() {
                        w.enqueue_ball_events(ball_script);
                    }
                    w.step(
                        physics_dt,
                        Some(SimStepInput {
                            shooter: &shooter,
                            shoot,
                            park,
                        }),
                    );
                    *physics_time.lock().expect("sim 시간") = w.sim_time;
                    catchup_steps += 1;
                    if catchup_steps >= MAX_CATCHUP_STEPS {
                        break;
                    }
                }

                thread::sleep(Duration::from_micros(500));
            }
        });

        info!(
            physics_hz = config.physics_hz,
            frame_hz = config.frame_hz,
            time_scale = config.time_scale,
            "Rapier sim 세션 시작 (슈터 + 로봇)"
        );

        return Self {
            world,
            clock,
            shutdown,
            physics_handle: Some(physics_handle),
            config,
            controls,
        };
    }

    /// 가상 카메라 소스를 만든다. `frames == 0` 이면 종료 신호까지 무한.
    pub fn camera(&self, camera_id: crate::CameraId, frames: u64) -> SimCamera {
        return SimCamera::new(
            camera_id,
            self.config.camera_count,
            frames,
            self.config.frame_hz,
            Arc::clone(&self.world),
            Arc::clone(&self.clock),
            Arc::clone(&self.shutdown),
        );
    }

    /// sim `Hardware` 어댑터를 만든다.
    pub fn hardware(&self) -> SimHardware {
        return SimHardware::new(Arc::clone(&self.world));
    }

    /// 공유 월드 핸들.
    pub fn world(&self) -> Arc<Mutex<SimWorld>> {
        return Arc::clone(&self.world);
    }

    /// GUI·발사 제어.
    pub fn controls(&self) -> Arc<Mutex<SimRuntimeControls>> {
        return Arc::clone(&self.controls);
    }

    /// 종료 플래그.
    pub fn shutdown_flag(&self) -> Arc<AtomicBool> {
        return Arc::clone(&self.shutdown);
    }

    /// 물리 스레드를 종료하고 join한다.
    pub fn shutdown(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        if let Some(handle) = self.physics_handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for SimSession {
    fn drop(&mut self) {
        self.shutdown();
    }
}
