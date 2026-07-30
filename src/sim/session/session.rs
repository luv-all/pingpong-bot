//! Rapier 디지털 트윈 세션.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use tracing::info;

use super::clock_handle::SimClockHandle;
use super::config::SimSessionConfig;
use super::controls::SimRuntimeControls;
use crate::camera;
use crate::camera::SimCamera;
use crate::hardware::SimHardware;
use crate::robot::Robot;
use crate::sim::physics::world::{SimStepInput, SimWorld};

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
    /// `robot` — sim·real·제어가 공유하는 불변 로봇 모델 (plan §2).
    pub fn new(
        config: SimSessionConfig,
        robot: Robot,
        controls: Arc<Mutex<SimRuntimeControls>>,
        shutdown: Arc<AtomicBool>,
    ) -> Self {
        return Self::with_physics(
            config,
            robot,
            controls,
            shutdown,
            crate::defaults::PhysicsParams::default(),
        );
    }

    /// config `[physics]`를 Rapier 월드에 반영한다.
    pub fn with_physics(
        config: SimSessionConfig,
        robot: Robot,
        controls: Arc<Mutex<SimRuntimeControls>>,
        shutdown: Arc<AtomicBool>,
        physics: crate::defaults::PhysicsParams,
    ) -> Self {
        let world = Arc::new(Mutex::new(SimWorld::with_physics(robot, physics)));
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
            // wall 경과 × scale로 절대 목표를 잡으면 배속 변경 시 목표가 튀어
            // (느리게 하면 sim이 멈춰 버리고, 빠르게 하면 거대한 catch-up 부채가 생긴다).
            // wall Δt × 현재 scale만큼만 sim을 진행한다.
            let mut last_wall = Instant::now();
            let mut sim_debt = 0.0_f64;
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

                let now = Instant::now();
                // 디버거 정지 등으로 wall이 커지면 한 번에 폭주하지 않게 캡.
                let wall_dt = now
                    .saturating_duration_since(last_wall)
                    .as_secs_f64()
                    .min(0.05);
                last_wall = now;
                sim_debt += wall_dt * time_scale;

                // 고배속에서도 따라갈 수 있게 스텝 상한을 scale에 비례.
                let max_catchup = ((8.0 * time_scale).ceil() as u32).clamp(8, 256);
                let mut catchup_steps = 0_u32;
                while sim_debt >= physics_dt && catchup_steps < max_catchup {
                    if physics_shutdown.load(Ordering::Acquire) {
                        return;
                    }
                    let (shoot, park, shooter, use_bang_bang_swing, rail_frame) = {
                        let mut ctrl = physics_controls.lock().expect("sim controls");
                        let shoot = ctrl.shoot_requested;
                        let park = ctrl.park_requested;
                        ctrl.shoot_requested = false;
                        ctrl.park_requested = false;
                        (
                            shoot,
                            park,
                            ctrl.shooter.clone(),
                            ctrl.use_bang_bang_swing,
                            ctrl.rail_frame,
                        )
                    };
                    let mut w = physics_world.lock().expect("sim 월드");
                    w.set_use_bang_bang_swing(use_bang_bang_swing);
                    w.step(
                        physics_dt,
                        Some(SimStepInput {
                            shooter: &shooter,
                            shoot,
                            park,
                            rail_frame,
                        }),
                    );
                    *physics_time.lock().expect("sim 시간") = w.sim_time;
                    sim_debt -= physics_dt;
                    catchup_steps += 1;
                }
                // 계속 밀리면 오래된 부채는 버리고 실시간성 유지.
                if sim_debt > physics_dt * f64::from(max_catchup) {
                    sim_debt = physics_dt * f64::from(max_catchup);
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
    pub fn camera(&self, camera_id: camera::Id, frames: u64) -> SimCamera {
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
