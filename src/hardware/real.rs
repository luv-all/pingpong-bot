//! Dynamixel 4축 실물 하드웨어 어댑터와 선택적 AXL 레일 동기 재생.

use crate::robot;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[cfg(not(all(windows, feature = "real")))]
use tracing::warn;
use tracing::{debug, error};

use super::dynamixel::{DynamixelBus, DynamixelConfig};
use super::rail::AxlRail;
use super::rail::RailConfig;
use crate::error::HwError;
use crate::hardware::{AppliedRailRacketCommand, Hardware};
use crate::robot::motion;

/// Dynamixel 버스와 quintic 재생 worker를 소유한다.
pub struct RealHardware {
    bus: Arc<Mutex<DynamixelBus>>,
    /// `None`이면 `rail_x = 0` (레일 비활성). executor와 pose 읽기가 공유.
    rail: Arc<Mutex<Option<AxlRail>>>,
    busy: Arc<AtomicBool>,
    cancel: Arc<AtomicBool>,
    executor: Option<JoinHandle<()>>,
    stream_hz: f64,
}

impl RealHardware {
    /// 실제 시리얼 포트를 열고 motion profile과 torque를 설정한다.
    pub fn new(config: DynamixelConfig, rail: Option<RailConfig>) -> Result<Self, HwError> {
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::open(config)?;
        bus.configure_position_mode_max_effort()?;
        bus.enable_torque(true)?;
        // 실포트: is_dry_run = false → AXL 실개방
        return Self::from_bus(bus, stream_hz, rail, false);
    }

    /// 포트를 열지 않지만 실제 좌표 변환·리밋·executor 경로를 그대로 사용한다.
    pub fn dry_run(config: DynamixelConfig, rail: Option<RailConfig>) -> Result<Self, HwError> {
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::dry_run(config).map_err(|e| HwError::InvalidConfig {
            reason: e.to_string(),
        })?;
        bus.configure_position_mode_max_effort()?;
        bus.enable_torque(true)?;
        return Self::from_bus(bus, stream_hz, rail, true);
    }

    fn from_bus(
        bus: DynamixelBus,
        stream_hz: f64,
        rail: Option<RailConfig>,
        is_dry_run: bool,
    ) -> Result<Self, HwError> {
        let rail = match rail.filter(|config| config.enabled) {
            None => {
                debug!("레일 비활성 — rail_x=0 고정");
                None
            }
            Some(config) if is_dry_run => {
                debug!(
                    dll = %config.dll_path.display(),
                    axis = config.axis,
                    "레일 dry-run"
                );
                Some(AxlRail::dry_run(config)?)
            }
            Some(config) => {
                #[cfg(all(windows, feature = "real"))]
                {
                    debug!(
                        dll = %config.dll_path.display(),
                        axis = config.axis,
                        irq_no = config.irq_no,
                        reverse = config.reverse,
                        x_min_m = config.x_min_m,
                        x_max_m = config.x_max_m,
                        "레일 Live 개방"
                    );
                    Some(AxlRail::open(config)?)
                }
                #[cfg(not(all(windows, feature = "real")))]
                {
                    warn!(
                        dll = %config.dll_path.display(),
                        axis = config.axis,
                        "AXL 레일은 Windows + feature=real 에서만 지원 — 레일 비활성, Dynamixel만 사용 (rail_x=0)"
                    );
                    None
                }
            }
        };
        return Ok(Self {
            bus: Arc::new(Mutex::new(bus)),
            rail: Arc::new(Mutex::new(rail)),
            busy: Arc::new(AtomicBool::new(false)),
            cancel: Arc::new(AtomicBool::new(false)),
            executor: None,
            stream_hz,
        });
    }

    fn read_rail_x_m(&mut self) -> Result<f64, HwError> {
        let mut guard = self.rail.lock().map_err(|_| HwError::ReadFailed {
            reason: "레일 mutex poisoned".into(),
        })?;
        return match guard.as_mut() {
            None => Ok(0.0),
            Some(rail) => rail.read_x_m(),
        };
    }

    fn reap_executor(&mut self) {
        if self.busy.load(Ordering::Acquire) {
            return;
        }
        if let Some(handle) = self.executor.take()
            && handle.join().is_err()
        {
            error!("Dynamixel 궤적 executor 패닉");
        }
    }

    /// 레일 이동을 시작하고 Dynamixel quintic 스트리밍 워커를 띄운다.
    /// 반환값은 AXL이 clamp한 실제 레일 목표다.
    fn start_trajectory(&mut self, trajectory: &motion::Trajectory) -> Result<f64, HwError> {
        self.reap_executor();
        if self.busy.swap(true, Ordering::AcqRel) {
            return Err(HwError::CommandFailed {
                duration_secs: trajectory.duration_secs,
                joint_count: trajectory.end.values.len(),
                reason: "이전 궤적이 아직 실행 중입니다".into(),
            });
        }

        let rail_result = (|| {
            let mut rail = self.rail.lock().map_err(|_| HwError::CommandFailed {
                duration_secs: trajectory.duration_secs,
                joint_count: trajectory.end.values.len(),
                reason: "레일 mutex poisoned".into(),
            })?;
            return match rail.as_mut() {
                Some(rail) => rail.command_abs_in_secs(
                    trajectory.follow_through_rail_x,
                    trajectory.duration_secs,
                ),
                None => Ok(0.0),
            };
        })();
        let applied_rail_m = match rail_result {
            Ok(value) => value,
            Err(error) => {
                self.busy.store(false, Ordering::Release);
                return Err(error);
            }
        };

        let trajectory = trajectory.clone();
        let bus = Arc::clone(&self.bus);
        let rail = Arc::clone(&self.rail);
        let busy = Arc::clone(&self.busy);
        self.cancel.store(false, Ordering::Release);
        let cancel = Arc::clone(&self.cancel);
        let tick = Duration::from_secs_f64(1.0 / self.stream_hz);
        self.executor = Some(thread::spawn(move || {
            let started = Instant::now();
            loop {
                if cancel.load(Ordering::Acquire) {
                    break;
                }
                let elapsed = started.elapsed().as_secs_f64();
                let sample_time = elapsed.min(trajectory.duration_secs);
                let joints = trajectory.sample_at(sample_time);
                let joints_ok = match bus.lock() {
                    Ok(mut bus) => match bus.write_joints(&joints) {
                        Ok(()) => true,
                        Err(error) => {
                            error!(sample_time, error = %error, "Dynamixel goal position 전송 실패 — 궤적 중단");
                            false
                        }
                    },
                    Err(_) => {
                        error!(sample_time, "Dynamixel bus mutex poisoned — 궤적 중단");
                        false
                    }
                };
                if !joints_ok || elapsed >= trajectory.duration_secs {
                    break;
                }
                thread::sleep(tick);
            }
            if !cancel.load(Ordering::Acquire)
                && let Ok(mut guard) = rail.lock()
                && let Some(rail_hw) = guard.as_mut()
                && let Err(error) = rail_hw.wait_idle()
            {
                error!(error = %error, "AXL 레일 완료 대기 실패");
            }
            busy.store(false, Ordering::Release);
        }));
        return Ok(applied_rail_m);
    }
}

impl Hardware for RealHardware {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError> {
        return self.start_trajectory(trajectory).map(|_| ());
    }

    fn read_pose(&mut self) -> Result<robot::Pose, HwError> {
        self.reap_executor();
        let joints = self
            .bus
            .lock()
            .map_err(|_| HwError::ReadFailed {
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .read_joints()?;
        return Ok(robot::Pose::new(self.read_rail_x_m()?, joints));
    }

    fn command_rail_and_racket(
        &mut self,
        trajectory: &motion::Trajectory,
    ) -> Result<AppliedRailRacketCommand, HwError> {
        let applied_joints = self
            .bus
            .lock()
            .map_err(|_| HwError::CommandFailed {
                duration_secs: trajectory.duration_secs,
                joint_count: trajectory.end.values.len(),
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .quantize_joints(&trajectory.end)?;
        let mut applied_trajectory = trajectory.clone();
        applied_trajectory.end = applied_joints.clone();
        applied_trajectory.follow_through = applied_joints.clone();
        let applied_rail_m = self.start_trajectory(&applied_trajectory)?;
        return Ok(AppliedRailRacketCommand {
            rail_m: applied_rail_m,
            joints: applied_joints,
            rail_sent: self.rail.lock().is_ok_and(|rail| rail.is_some()),
        });
    }

    fn is_busy(&mut self) -> bool {
        self.reap_executor();
        return self.busy.load(Ordering::Acquire);
    }

    fn cancel(&mut self) {
        // 다음 명령을 즉시 시작할 수 있도록 executor 종료를 기다린 뒤
        // 실측 관절을 다시 명령해 현재 위치를 홀드한다.
        self.cancel.store(true, Ordering::Release);
        if let Ok(mut rail) = self.rail.lock()
            && let Some(rail) = rail.as_mut()
            && let Err(error) = rail.stop()
        {
            error!(%error, "AXL 레일 안전 정지 실패");
        }
        if let Some(handle) = self.executor.take() {
            let _ = handle.join();
        }
        self.busy.store(false, Ordering::Release);
        if let Ok(mut bus) = self.bus.lock()
            && let Ok(joints) = bus.read_joints()
        {
            let _ = bus.write_joints(&joints);
        }
    }
}

impl Drop for RealHardware {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Release);
        self.reap_executor();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hardware::dynamixel::DynamixelConfig;
    use crate::hardware::rail::RailConfig;
    use crate::robot::Joints;

    fn test_rail() -> RailConfig {
        return RailConfig {
            enabled: true,
            dll_path: std::path::PathBuf::from("dummy.dll"),
            ..RailConfig::default()
        };
    }

    #[test]
    fn dry_run_read_pose_uses_rail_position() {
        let dynamixel = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(dynamixel, Some(test_rail())).expect("dry-run hardware");

        assert_eq!(hardware.read_pose().expect("pose").rail_x, 0.0);
    }

    /// non-Windows live 레일은 soft-skip → `rail_x=0` (Dynamixel 버스는 dry로 대체).
    #[cfg(not(all(windows, feature = "real")))]
    #[test]
    fn non_windows_live_rail_soft_skips_to_rail_x_zero() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::dry_run(config).expect("dry bus");
        bus.configure_position_mode_max_effort()
            .expect("position mode");
        bus.enable_torque(true).expect("torque");
        let mut hardware =
            RealHardware::from_bus(bus, stream_hz, Some(test_rail()), false).expect("hardware");
        assert_eq!(hardware.read_pose().expect("pose").rail_x, 0.0);
    }

    #[test]
    fn dry_run_executes_trajectory_and_reports_busy_state() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware = RealHardware::dry_run(config, None).expect("dry-run hardware");
        let trajectory = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.1; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.03,
            motion::Rail::fixed(0.0),
        );

        hardware.command(&trajectory).expect("command");
        assert!(hardware.is_busy());
        thread::sleep(Duration::from_millis(80));
        assert!(!hardware.is_busy());

        let pose = hardware.read_pose().expect("pose");
        assert_eq!(pose.rail_x, 0.0);
        for angle in pose.joints.values {
            assert!((angle - 0.1).abs() < 0.002);
        }
    }

    #[test]
    fn cancel_allows_an_immediate_replacement_trajectory() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware = RealHardware::dry_run(config, None).expect("dry-run hardware");
        let first = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.2; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.2,
            motion::Rail::fixed(0.0),
        );
        let replacement = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[-0.1; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.02,
            motion::Rail::fixed(0.0),
        );

        hardware.command(&first).expect("first command");
        hardware.cancel();
        hardware.command(&replacement).expect("replacement command");
        while hardware.is_busy() {
            thread::sleep(Duration::from_millis(5));
        }

        let pose = hardware.read_pose().expect("pose");
        for angle in pose.joints.values {
            assert!((angle - -0.1).abs() < 0.002);
        }
    }

    #[test]
    fn dry_run_syncs_rail_with_joint_trajectory() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        let trajectory = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.05; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.04,
            motion::Rail {
                start: 0.0,
                end: 0.25,
                start_velocity: 0.0,
                end_velocity: 0.0,
            },
        );

        hardware.command(&trajectory).expect("command");
        thread::sleep(Duration::from_millis(100));
        assert!(!hardware.is_busy());

        let pose = hardware.read_pose().expect("pose");
        assert!((pose.rail_x - 0.25).abs() < 1e-9);
        for angle in pose.joints.values {
            assert!((angle - 0.05).abs() < 0.002);
        }
    }

    #[test]
    fn direct_tracking_applies_rail_and_full_racket_pose() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        let before = hardware.read_pose().expect("before");
        let target = Joints::from_slice(&[0.10, -0.15, 0.20, -0.25]);
        let trajectory = motion::Trajectory::new(
            before.joints,
            target.clone(),
            vec![0.0; target.values.len()],
            vec![0.0; target.values.len()],
            0.1,
            motion::Rail {
                start: before.rail_x,
                end: 0.35,
                start_velocity: 0.0,
                end_velocity: 0.0,
            },
        );

        let applied = hardware
            .command_rail_and_racket(&trajectory)
            .expect("tracking command");
        assert!((applied.rail_m - 0.35).abs() < 1e-9);
        for (actual, requested) in applied.joints.values.iter().zip(&target.values) {
            assert!((actual - requested).abs() < 0.002);
        }
        assert!(applied.rail_sent);

        while hardware.is_busy() {
            thread::sleep(Duration::from_millis(5));
        }
        let after = hardware.read_pose().expect("after");
        assert!((after.rail_x - 0.35).abs() < 1e-9);
        for (actual, requested) in after.joints.values.iter().zip(&target.values) {
            assert!((actual - requested).abs() < 0.002);
        }
    }

    #[test]
    fn drop_cancels_long_running_trajectory_promptly() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware = RealHardware::dry_run(config, None).expect("dry-run hardware");
        let trajectory = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.1; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            2.0,
            motion::Rail::fixed(0.0),
        );
        hardware.command(&trajectory).expect("command");

        let started = std::time::Instant::now();
        drop(hardware);

        assert!(started.elapsed() < Duration::from_millis(100));
    }
}
