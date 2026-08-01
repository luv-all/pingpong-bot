//! Dynamixel 4축 실물 하드웨어 어댑터와 선택적 AXL 레일 동기 재생.

use crate::robot;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[cfg(not(all(windows, feature = "real")))]
use tracing::warn;
use tracing::{debug, error, info};

use super::dynamixel::{DynamixelBus, DynamixelConfig};
use super::rail::AxlRail;
use super::rail::RailConfig;
use crate::defaults;
use crate::error::HwError;
use crate::hardware::Hardware;
use crate::robot::Arm;
use crate::robot::motion;

/// Dynamixel 버스와 quintic 재생 worker를 소유한다.
pub struct RealHardware {
    bus: Arc<Mutex<DynamixelBus>>,
    /// 레일 dry-run·RNEA 디버그용 Arm 핸들 (스윙 실행 자체는 Goal Position만 씀).
    #[allow(dead_code)]
    arm: Arc<Arm>,
    /// `None`이면 `rail_x = 0` (레일 비활성). executor와 pose 읽기가 공유.
    rail: Arc<Mutex<Option<AxlRail>>>,
    busy: Arc<AtomicBool>,
    cancel: Arc<AtomicBool>,
    executor: Option<JoinHandle<()>>,
    stream_hz: f64,
    /// 2단계 제어 중 마지막으로 보낸 레일 목표. 손목만 갱신할 때 진행 중인
    /// 레일을 매번 정지·재시작하지 않기 위한 중복 억제값이다.
    direct_rail_target: Option<f64>,
}

impl RealHardware {
    /// 실제 시리얼 포트를 열고 motion profile과 torque를 설정한다.
    pub fn new(
        config: DynamixelConfig,
        rail: Option<RailConfig>,
        arm: Arc<Arm>,
    ) -> Result<Self, HwError> {
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::open(config)?;
        // 실기 운전 때마다 Operating Mode/PWM/Current Limit EEPROM을 다시 읽고
        // 검사하지 않는다. 단순 2단계 제어에는 필요 없고, 일부 MX-28(ID 4)의
        // EEPROM 응답 체크섬 오류가 토크 락 이전에 전체 초기화를 막았다.
        // 벤치 기본 설정은 유지하고 RAM Goal/토크만 초기화한다.
        // 단순 제어 시험은 시작 즉시 기본 자세로 이동한다. 불안정한 Present Position
        // Status Packet을 먼저 읽지 않고, 기본 Goal을 기록한 뒤 Torque ON 한다.
        // 첫 실행과 이전 실행에서 토크가 남은 재실행이 동일한 경로를 탄다.
        bus.lock_at_joints(&arm.default_joints)?;
        // 실포트: is_dry_run = false → AXL 실개방
        return Self::from_bus(bus, stream_hz, rail, false, arm);
    }

    /// 포트를 열지 않지만 실제 좌표 변환·리밋·executor 경로를 그대로 사용한다.
    pub fn dry_run(config: DynamixelConfig, rail: Option<RailConfig>) -> Result<Self, HwError> {
        return Self::dry_run_with_arm(
            config,
            rail,
            Arc::new(
                (*defaults::urdf_4dof()
                    .map_err(|e| HwError::InvalidConfig {
                        reason: e.to_string(),
                    })?
                    .arm)
                    .clone(),
            ),
        );
    }

    /// dry-run + 명시적 Arm (RNEA FF 테스트용).
    pub fn dry_run_with_arm(
        config: DynamixelConfig,
        rail: Option<RailConfig>,
        arm: Arc<Arm>,
    ) -> Result<Self, HwError> {
        let stream_hz = config.stream_hz;
        let mut bus = DynamixelBus::dry_run(config).map_err(|e| HwError::InvalidConfig {
            reason: e.to_string(),
        })?;
        bus.configure_position_mode_max_effort()?;
        bus.lock_current_position()?;
        return Self::from_bus(bus, stream_hz, rail, true, arm);
    }

    fn from_bus(
        bus: DynamixelBus,
        stream_hz: f64,
        rail: Option<RailConfig>,
        is_dry_run: bool,
        arm: Arc<Arm>,
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
            arm,
            rail: Arc::new(Mutex::new(rail)),
            busy: Arc::new(AtomicBool::new(false)),
            cancel: Arc::new(AtomicBool::new(false)),
            executor: None,
            stream_hz,
            direct_rail_target: None,
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
            error!("Dynamixel swing executor 패닉");
        }
    }
}

impl Hardware for RealHardware {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError> {
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
        let rail_target = trajectory.follow_through_rail_x;
        let rail_duration = trajectory.duration_secs;
        self.executor = Some(thread::spawn(move || {
            if let Ok(mut guard) = rail.lock()
                && let Some(rail_hw) = guard.as_mut()
                && let Err(error) = rail_hw.command_abs_in_secs(rail_target, rail_duration)
            {
                error!(
                    rail_target,
                    rail_duration,
                    error = %error,
                    "AXL 레일 이동 시작 실패 — 스윙 중단"
                );
                busy.store(false, Ordering::Release);
                return;
            }

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
                            error!(
                                sample_time,
                                error = %error,
                                "Dynamixel goal position 전송 실패 — 스윙 중단"
                            );
                            false
                        }
                    },
                    Err(_) => {
                        error!(sample_time, "Dynamixel bus mutex poisoned — 스윙 중단");
                        false
                    }
                };
                if !joints_ok {
                    break;
                }

                if elapsed >= trajectory.duration_secs {
                    break;
                }
                thread::sleep(tick);
            }
            // 정상 완주에서는 관절뿐 아니라 레일까지 실제 정지해야 명령 완료다.
            // 취소된 경우에는 다음 정밀 명령이 AxmMoveSStop 후 즉시 재목표하므로 기다리지 않는다.
            if !cancel.load(Ordering::Acquire)
                && let Ok(mut guard) = rail.lock()
                && let Some(rail_hw) = guard.as_mut()
                && let Err(error) = rail_hw.wait_idle()
            {
                error!(error = %error, "AXL 레일 완료 대기 실패");
            }
            busy.store(false, Ordering::Release);
        }));
        return Ok(());
    }

    fn read_pose(&mut self) -> Result<robot::Pose, HwError> {
        self.reap_executor();
        // 이번 2단계 실기 시험에서는 Dynamixel을 반복해서 읽지 않는다. 초기 토크 락 때
        // 검증한 자세와 이후 보낸 Goal을 사용해 센터 이동/관전 포즈만 구성한다.
        let joints = self
            .bus
            .lock()
            .map_err(|_| HwError::ReadFailed {
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .cached_joints()?;
        return Ok(robot::Pose::new(self.read_rail_x_m()?, joints));
    }

    fn command_initial_pose(&mut self, rail_x: f64, joints: &robot::Joints) -> Result<(), HwError> {
        self.reap_executor();
        if self.busy.load(Ordering::Acquire) {
            return Err(HwError::CommandFailed {
                duration_secs: 0.0,
                joint_count: joints.values.len(),
                reason: "다른 하드웨어 명령이 실행 중입니다".into(),
            });
        }

        // DYNAMIXEL의 내부 Profile Acceleration/Velocity가 기본 자세까지 부드럽게
        // 이동시키므로 Goal을 한 번만 보낸다. 반복 Status Packet read는 하지 않는다.
        self.bus
            .lock()
            .map_err(|_| HwError::CommandFailed {
                duration_secs: 0.0,
                joint_count: joints.values.len(),
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .write_joints(joints)?;
        info!(goal = ?joints.values, "Dynamixel 기본 자세 이동 명령");

        // 최악 관절 이동에도 내부 프로파일이 도달할 시간을 준 뒤 레일을 움직인다.
        // 팔을 먼저 접어 둬 레일 이동 중 테이블/주변물과 간섭할 가능성을 줄인다.
        thread::sleep(Duration::from_secs(3));

        let mut rail = self.rail.lock().map_err(|_| HwError::CommandFailed {
            duration_secs: 0.0,
            joint_count: joints.values.len(),
            reason: "레일 mutex poisoned".into(),
        })?;
        if let Some(rail) = rail.as_mut() {
            // 끝에서 끝까지여도 약 0.7 m/s 이하가 되도록 2초 기준으로 이동시키고
            // 실제 정지가 확인될 때까지 Ready로 넘어가지 않는다.
            let commanded = rail.command_abs_in_secs(rail_x, 2.0)?;
            rail.wait_idle()?;
            self.direct_rail_target = Some(commanded);
            info!(rail_x = commanded, "AXL 레일 중앙 정렬 완료");
        }
        return Ok(());
    }

    fn command_rail_and_racket(
        &mut self,
        rail_x: f64,
        racket_joint_rad: f64,
        duration_secs: f64,
    ) -> Result<(), HwError> {
        self.reap_executor();
        if self.busy.load(Ordering::Acquire) {
            return Err(HwError::CommandFailed {
                duration_secs,
                joint_count: 1,
                reason: "중앙 이동 궤적이 아직 실행 중입니다".into(),
            });
        }

        if self
            .direct_rail_target
            .is_none_or(|last| (last - rail_x).abs() >= 0.01)
        {
            let mut rail = self.rail.lock().map_err(|_| HwError::CommandFailed {
                duration_secs,
                joint_count: 1,
                reason: "레일 mutex poisoned".into(),
            })?;
            if let Some(rail) = rail.as_mut() {
                rail.command_abs_in_secs(rail_x, duration_secs)?;
            }
            self.direct_rail_target = Some(rail_x);
        }
        // 4-DOF 배선의 마지막 논리 관절은 라켓 손목(ID 5)이다. 이 호출은
        // ID 1/2/3/4에 Goal Position을 보내지 않는다.
        self.bus
            .lock()
            .map_err(|_| HwError::CommandFailed {
                duration_secs,
                joint_count: 1,
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .write_joint(3, racket_joint_rad)?;
        return Ok(());
    }

    fn is_busy(&mut self) -> bool {
        self.reap_executor();
        return self.busy.load(Ordering::Acquire);
    }

    fn cancel(&mut self) {
        // executor 루프가 매 틱 이 플래그를 보고 빠져나오며 `busy`를 내린다.
        // `Drop`이 쓰는 것과 같은 경로다.
        self.cancel.store(true, Ordering::Release);
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
        let arm = Arc::new((*crate::defaults::urdf_4dof().expect("urdf").arm).clone());
        let mut hardware = RealHardware::from_bus(bus, stream_hz, Some(test_rail()), false, arm)
            .expect("hardware");
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
    fn direct_tracking_changes_only_rail_and_racket_joint() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        let before = hardware.read_pose().expect("before");

        hardware
            .command_rail_and_racket(0.35, -0.25, 0.1)
            .expect("tracking command");

        let after = hardware.read_pose().expect("after");
        assert!((after.rail_x - 0.35).abs() < 1e-9);
        for index in 0..3 {
            assert!((after.joints.values[index] - before.joints.values[index]).abs() < 0.002);
        }
        assert!((after.joints.values[3] - -0.25).abs() < 0.002);
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
