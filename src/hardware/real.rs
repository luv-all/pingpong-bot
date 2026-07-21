//! Dynamixel 4축 실물 하드웨어 어댑터. AXL 레일은 x=0 스텁이다.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use tracing::{debug, error, warn};

use super::dynamixel::{DynamixelBus, DynamixelConfig};
use super::rail_stub::RailStub;
use crate::{Hardware, HwError, RobotPose, SwingTrajectory};

/// Dynamixel 버스와 quintic 재생 worker를 소유한다.
pub struct RealHardware {
    bus: Arc<Mutex<DynamixelBus>>,
    rail: RailStub,
    busy: Arc<AtomicBool>,
    cancel: Arc<AtomicBool>,
    executor: Option<JoinHandle<()>>,
    stream_hz: f64,
}

impl RealHardware {
    /// 실제 시리얼 포트를 열고 motion profile과 torque를 설정한다.
    pub fn new(config: DynamixelConfig) -> Result<Self, HwError> {
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::open(config)?;
        bus.enable_torque(true)?;
        return Ok(Self::from_bus(bus, stream_hz));
    }

    /// 포트를 열지 않지만 실제 좌표 변환·리밋·executor 경로를 그대로 사용한다.
    pub fn dry_run(config: DynamixelConfig) -> Result<Self, HwError> {
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::dry_run(config).map_err(|e| HwError::InvalidConfig {
            reason: e.to_string(),
        })?;
        bus.enable_torque(true)?;
        return Ok(Self::from_bus(bus, stream_hz));
    }

    fn from_bus(bus: DynamixelBus, stream_hz: f64) -> Self {
        return Self {
            bus: Arc::new(Mutex::new(bus)),
            rail: RailStub,
            busy: Arc::new(AtomicBool::new(false)),
            cancel: Arc::new(AtomicBool::new(false)),
            executor: None,
            stream_hz,
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
        if (trajectory.rail.end - trajectory.rail.start).abs() > 1e-4
            || trajectory.rail.end_velocity.abs() > 1e-4
            || (trajectory.follow_through_rail_x - trajectory.rail.end).abs() > 1e-4
        {
            warn!("AXL 레일 미구현 — rail_x=0으로 Dynamixel 관절 궤적만 실행");
        }

        let trajectory = trajectory.clone();
        let bus = Arc::clone(&self.bus);
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
                let result = bus
                    .lock()
                    .map_err(|_| ())
                    .and_then(|mut bus| bus.write_joints(&joints).map_err(|_| ()));
                if result.is_err() {
                    error!(sample_time, "Dynamixel goal position 전송 실패 — 스윙 중단");
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
        return Ok(RobotPose::new(self.rail.read_x(), joints));
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
    use std::thread;
    use std::time::Duration;

    use crate::{Hardware, Joints, RailMotion, SwingTrajectory};

    use super::RealHardware;
    use crate::hardware::dynamixel::DynamixelConfig;

    #[test]
    fn dry_run_executes_trajectory_and_reports_busy_state() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware = RealHardware::dry_run(config).expect("dry-run hardware");
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
    fn drop_cancels_long_running_trajectory_promptly() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware = RealHardware::dry_run(config).expect("dry-run hardware");
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
