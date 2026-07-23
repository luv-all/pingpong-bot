//! Dynamixel 4축 실물 하드웨어 어댑터와 선택적 AXL 레일 동기 재생.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use tracing::{debug, error};

use super::axl_rail::AxlRail;
use super::dynamixel::{DynamixelBus, DynamixelConfig};
use super::rail::RailConfig;
use crate::{Hardware, HwError, RobotPose, SwingTrajectory};

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
        bus.enable_torque(true)?;
        return Self::from_bus(bus, stream_hz, rail, false);
    }

    /// 포트를 열지 않지만 실제 좌표 변환·리밋·executor 경로를 그대로 사용한다.
    pub fn dry_run(config: DynamixelConfig, rail: Option<RailConfig>) -> Result<Self, HwError> {
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::dry_run(config).map_err(|e| HwError::InvalidConfig {
            reason: e.to_string(),
        })?;
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
            None => None,
            Some(config) if is_dry_run => Some(AxlRail::dry_run(config)?),
            Some(config) => Some(AxlRail::open(config)?),
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
        let mut guard = self.rail.lock().map_err(|_| HwError::ReadFailed)?;
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
            error!("Dynamixel swing executor 패닉");
        }
    }
}

impl Hardware for RealHardware {
    fn command(&mut self, trajectory: &SwingTrajectory) -> Result<(), HwError> {
        self.reap_executor();
        if self.busy.swap(true, Ordering::AcqRel) {
            debug!("Dynamixel 스윙 실행 중 — 중복 명령 무시");
            return Ok(());
        }

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
                let rail_x = trajectory.sample_rail_at(sample_time);

                let joints_ok = bus
                    .lock()
                    .map_err(|_| ())
                    .and_then(|mut bus| bus.write_joints(&joints).map_err(|_| ()))
                    .is_ok();
                if !joints_ok {
                    error!(sample_time, "Dynamixel goal position 전송 실패 — 스윙 중단");
                    break;
                }

                if let Ok(mut guard) = rail.lock()
                    && let Some(rail_hw) = guard.as_mut()
                    && rail_hw.command_abs_m(rail_x).is_err()
                {
                    error!(sample_time, rail_x, "AXL 레일 목표 전송 실패 — 스윙 중단");
                    break;
                }

                if elapsed >= trajectory.duration_secs {
                    break;
                }
                thread::sleep(tick);
            }
            busy.store(false, Ordering::Release);
        }));
        return Ok(());
    }

    fn read_pose(&mut self) -> Result<RobotPose, HwError> {
        self.reap_executor();
        let joints = self
            .bus
            .lock()
            .map_err(|_| HwError::ReadFailed)?
            .read_joints()?;
        return Ok(RobotPose::new(self.read_rail_x_m()?, joints));
    }

    fn is_busy(&mut self) -> bool {
        self.reap_executor();
        return self.busy.load(Ordering::Acquire);
    }
}

impl Drop for RealHardware {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Release);
        if let Some(handle) = self.executor.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use std::thread;
    use std::time::Duration;

    use crate::{Hardware, Joints, RailMotion, SwingTrajectory};

    use super::RealHardware;
    use crate::defaults::dynamixel;
    use crate::hardware::dynamixel::DynamixelConfig;
    use crate::hardware::rail::RailConfig;

    fn test_rail() -> RailConfig {
        return RailConfig {
            enabled: true,
            dll_path: PathBuf::from("unused.dll"),
            pulses_per_meter: 1000,
            x_min_m: -1.0,
            x_max_m: 1.0,
            vel: 0.2,
            accel: 1.0,
            decel: 1.0,
            min_vel: 0.001,
            max_vel: 1.0,
            ..RailConfig::default()
        };
    }

    #[test]
    fn dry_run_read_pose_uses_rail_position() {
        let dynamixel = DynamixelConfig {
            stream_hz: 500.0,
            ..dynamixel()
        };
        let mut hardware =
            RealHardware::dry_run(dynamixel, Some(test_rail())).expect("dry-run hardware");

        assert_eq!(hardware.read_pose().expect("pose").rail_x, 0.0);
    }

    #[test]
    fn dry_run_executes_trajectory_and_reports_busy_state() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..crate::defaults::dynamixel()
        };
        let mut hardware = RealHardware::dry_run(config, None).expect("dry-run hardware");
        let trajectory = SwingTrajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.1; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.03,
            RailMotion::fixed(0.0),
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
    fn dry_run_syncs_rail_with_joint_trajectory() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..crate::defaults::dynamixel()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        let trajectory = SwingTrajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.05; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.04,
            RailMotion {
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
    fn drop_cancels_long_running_trajectory_promptly() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..crate::defaults::dynamixel()
        };
        let mut hardware = RealHardware::dry_run(config, None).expect("dry-run hardware");
        let trajectory = SwingTrajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.1; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            2.0,
            RailMotion::fixed(0.0),
        );
        hardware.command(&trajectory).expect("command");

        let started = std::time::Instant::now();
        drop(hardware);

        assert!(started.elapsed() < Duration::from_millis(100));
    }
}
