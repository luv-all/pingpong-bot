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
use super::rail::RailQueue;
use crate::error::HwError;
use crate::hardware::{AppliedRailRacketCommand, Hardware};
use crate::robot::motion;

/// Dynamixel 버스와 quintic 재생 worker를 소유한다.
pub struct RealHardware {
    bus: Arc<Mutex<DynamixelBus>>,
    /// `None`이면 `rail_x = 0` (레일 비활성). `RailQueue`가 AXL 드라이버를
    /// 배타적으로 소유하며 executor 스레드와 직접 호출부가 함께 공유한다.
    rail: Option<Arc<RailQueue<AxlRail>>>,
    /// `command_rail`/`command_rail_and_racket`이 실제로 명령을 보내지 않고도
    /// 클램프된 목표를 동기로 돌려주는 데 쓴다. `x_min_m`/`x_max_m`만 읽으며,
    /// 이 값들은 홈잉으로도 바뀌지 않으므로(바뀌는 건 `board_zero_domain_m`뿐)
    /// 생성 시점 복사본으로 계속 유효하다.
    rail_config: Option<RailConfig>,
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
        // Goal Position, Torque Enable, EEPROM 설정 중 어떤 것도 건드리기
        // 전에 듀얼 모터 기계 정렬을 검사한다. 예전에는 enable_torque가
        // ID2에 계산된 미러 목표를 먼저 쓴 뒤 여기서 실패해, 검사가
        // 급동작을 막지 못했다.
        bus.verify_mirror_alignment()?;
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
        let rail_config = rail.clone().filter(|config| config.enabled);
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
            rail: rail.map(|rail| Arc::new(RailQueue::spawn(rail))),
            rail_config,
            busy: Arc::new(AtomicBool::new(false)),
            cancel: Arc::new(AtomicBool::new(false)),
            executor: None,
            stream_hz,
        });
    }

    fn read_rail_x_m(&mut self) -> Result<f64, HwError> {
        return match &self.rail {
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

    fn start_trajectory(
        &mut self,
        trajectory: &motion::Trajectory,
        drive_rail: bool,
    ) -> Result<(), HwError> {
        self.reap_executor();
        if !drive_rail && self.busy.load(Ordering::Acquire) {
            // 타격 직전 q3 손목 명령은 진행 중인 전체 정렬의 관절
            // 스트림만 선점한다. AXL에 이미 내려간 절대 목표는 보드가
            // 계속 수행하므로 여기서 레일은 정지시키지 않는다.
            self.cancel.store(true, Ordering::Release);
            if let Some(handle) = self.executor.take()
                && handle.join().is_err()
            {
                error!("Dynamixel 궤적 executor 선점 중 패닉");
            }
            self.busy.store(false, Ordering::Release);
        }
        if self.busy.swap(true, Ordering::AcqRel) {
            debug!("Dynamixel 궤적 실행 중 — 중복 명령 무시");
            return Ok(());
        }

        let trajectory = trajectory.clone();
        let bus = Arc::clone(&self.bus);
        let rail = self.rail.clone();
        let busy = Arc::clone(&self.busy);
        self.cancel.store(false, Ordering::Release);
        let cancel = Arc::clone(&self.cancel);
        let tick = Duration::from_secs_f64(1.0 / self.stream_hz);
        let rail_target = trajectory.follow_through_rail_x;
        let rail_duration = trajectory.duration_secs;
        self.executor = Some(thread::spawn(move || {
            // enqueue는 전송 성공 여부를 동기로 알려주지 않는다 — 실패는
            // 아래 wait_idle 이후 take_error()로만 드러난다. 그때는 이미
            // 관절이 레일 목표 도착을 가정하고 스트리밍을 마쳤을 수 있다.
            if drive_rail
                && let Some(rail_queue) = &rail
            {
                rail_queue.enqueue(rail_target, rail_duration);
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
                                "Dynamixel goal position 전송 실패 — 궤적 중단"
                            );
                            false
                        }
                    },
                    Err(_) => {
                        error!(sample_time, "Dynamixel bus mutex poisoned — 궤적 중단");
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
            if drive_rail
                && !cancel.load(Ordering::Acquire)
                && let Some(rail_queue) = &rail
            {
                rail_queue.wait_idle();
                if let Some(error) = rail_queue.take_error() {
                    error!(
                        rail_target,
                        rail_duration,
                        %error,
                        "AXL 레일 이동 실패 — 팔은 이미 레일이 목표에 도착했다고 가정하고 진행했습니다"
                    );
                }
            }
            busy.store(false, Ordering::Release);
        }));
        return Ok(());
    }

    /// 온디맨드 레일 홈잉. `--calibrate-rail`과 jog 툴 버튼이 이 메서드를 부른다.
    /// 완료까지(최대 몇 분) 블로킹하며, 그동안 다른 모든 레일 요청도 함께 대기한다.
    pub fn home_rail(
        &mut self,
        end: super::rail::RailEnd,
    ) -> Result<super::rail::RailHomeResult, HwError> {
        return match &self.rail {
            None => Err(HwError::InvalidConfig {
                reason: "레일이 비활성화됨 — home_rail 호출 불가".into(),
            }),
            Some(rail) => rail.home(end),
        };
    }
}

impl Hardware for RealHardware {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError> {
        return self.start_trajectory(trajectory, true);
    }

    fn command_joints(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError> {
        return self.start_trajectory(trajectory, false);
    }

    fn command_rail(&mut self, rail_x: f64, duration_secs: f64) -> Result<f64, HwError> {
        self.reap_executor();
        return match (&self.rail, &self.rail_config) {
            (Some(rail), Some(config)) => {
                let clamped_m = config.clamp_m(rail_x);
                rail.enqueue(clamped_m, duration_secs);
                Ok(clamped_m)
            }
            _ => Ok(0.0),
        };
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

    fn arm_joint_limit_escape(&mut self, joints: &robot::Joints) -> Result<(), HwError> {
        self.reap_executor();
        self.bus
            .lock()
            .map_err(|_| HwError::CommandFailed {
                duration_secs: 0.0,
                joint_count: joints.values.len(),
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .arm_limit_escape_from(joints)
    }

    fn verify_coupled_joints(&mut self) -> Result<(), HwError> {
        self.reap_executor();
        self.bus
            .lock()
            .map_err(|_| HwError::ReadFailed {
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .verify_mirror_alignment()
    }

    fn command_rail_and_racket(
        &mut self,
        rail_x: f64,
        aim_joint_rad: f64,
        duration_secs: f64,
    ) -> Result<AppliedRailRacketCommand, HwError> {
        self.reap_executor();
        if self.busy.load(Ordering::Acquire) {
            return Err(HwError::CommandFailed {
                duration_secs,
                joint_count: 1,
                reason: "중앙 이동 궤적이 아직 실행 중입니다".into(),
            });
        }

        let (applied_rail_m, rail_sent) = match (&self.rail, &self.rail_config) {
            (Some(rail), Some(config)) => {
                let clamped_m = config.clamp_m(rail_x);
                rail.enqueue(clamped_m, duration_secs);
                (clamped_m, true)
            }
            _ => (0.0, false),
        };
        // 4-DOF 논리 관절 1번은 라켓 수평 조준축(ID 3)이다.
        // 다른 관절에는 Goal Position을 보내지 않는다.
        let applied_aim_rad = self
            .bus
            .lock()
            .map_err(|_| HwError::CommandFailed {
                duration_secs,
                joint_count: 1,
                reason: "Dynamixel bus mutex poisoned".into(),
            })?
            .write_joint(crate::robot::control::DIRECT_AIM_JOINT_INDEX, aim_joint_rad)?;
        return Ok(AppliedRailRacketCommand {
            rail_m: applied_rail_m,
            aim_rad: applied_aim_rad,
            rail_sent,
        });
    }

    fn is_busy(&mut self) -> bool {
        self.reap_executor();
        return self.busy.load(Ordering::Acquire);
    }

    fn log_joint_diagnostics(&mut self) {
        match self.bus.lock() {
            Ok(mut bus) => bus.log_joint_diagnostics(),
            Err(_) => error!("Dynamixel 진단 실패 — bus mutex poisoned"),
        }
    }

    fn recover_joint_control(&mut self) -> Result<bool, HwError> {
        return self
            .bus
            .lock()
            .map_err(|_| HwError::ReadFailed {
                reason: "Dynamixel 복구 실패 — bus mutex poisoned".into(),
            })?
            .recover_joint_control();
    }

    fn cancel(&mut self) {
        // executor 루프가 매 틱 이 플래그를 보고 빠져나오며 `busy`를 내린다.
        // `Drop`이 쓰는 것과 같은 경로다.
        self.cancel.store(true, Ordering::Release);
        // RailQueue::stop은 논블로킹이라 실패를 동기로 알 수 없다 — 실패하면
        // 다음 take_error() 폴에서 드러난다(다른 레일 호출부와 동일).
        if let Some(rail) = &self.rail {
            rail.stop();
        }
        if let Ok(mut bus) = self.bus.lock()
            && let Ok(joints) = bus.read_joints()
            && let Some(aim) = joints
                .values
                .get(crate::robot::control::DIRECT_AIM_JOINT_INDEX)
        {
            let _ = bus.write_joint(crate::robot::control::DIRECT_AIM_JOINT_INDEX, *aim);
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

    #[test]
    fn home_rail_errors_when_dry_run() {
        let dynamixel = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(dynamixel, Some(test_rail())).expect("dry-run hardware");
        assert!(
            hardware
                .home_rail(crate::hardware::rail::RailEnd::Min)
                .is_err()
        );
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
    fn dry_run_syncs_rail_with_joint_trajectory() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        let trajectory = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.10, 0.05, 0.05, 0.05]),
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
        assert!((pose.joints.values[0] - 0.10).abs() < 0.002);
        for angle in &pose.joints.values[1..] {
            assert!((*angle - 0.05).abs() < 0.002);
        }
    }

    #[test]
    fn joint_only_trajectory_does_not_overwrite_direct_rail_target() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        hardware
            .command_rail_and_racket(0.35, 0.0, 0.10)
            .expect("direct rail command");
        let trajectory = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.05; 4]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.04,
            motion::Rail::fixed(0.0),
        );

        hardware.command_joints(&trajectory).expect("joint command");
        thread::sleep(Duration::from_millis(100));

        let pose = hardware.read_pose().expect("pose");
        assert!((pose.rail_x - 0.35).abs() < 1e-9);
    }

    #[test]
    fn joint_only_trajectory_preempts_joint_stream_without_stopping_rail() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        let alignment = motion::Trajectory::new(
            Joints::from_slice(&[0.0; 4]),
            Joints::from_slice(&[0.20, 0.20, 0.20, 0.20]),
            vec![0.0; 4],
            vec![0.0; 4],
            0.30,
            motion::Rail {
                start: 0.0,
                end: 0.35,
                start_velocity: 0.0,
                end_velocity: 0.0,
            },
        );
        hardware.command(&alignment).expect("alignment command");
        thread::sleep(Duration::from_millis(20));

        let snap_start = hardware.read_pose().expect("snap start");
        let mut snap_end = snap_start.joints.clone();
        snap_end.values[3] += 0.10;
        let snap = motion::Trajectory::new(
            snap_start.joints.clone(),
            snap_end.clone(),
            vec![0.0; 4],
            vec![0.0; 4],
            0.04,
            motion::Rail::fixed(snap_start.rail_x),
        );

        hardware.command_joints(&snap).expect("wrist snap command");
        thread::sleep(Duration::from_millis(100));

        let pose = hardware.read_pose().expect("pose");
        assert!((pose.rail_x - 0.35).abs() < 1e-9);
        for index in 0..3 {
            assert!((pose.joints.values[index] - snap_start.joints.values[index]).abs() < 0.002);
        }
        assert!((pose.joints.values[3] - snap_end.values[3]).abs() < 0.002);
    }

    #[test]
    fn direct_tracking_changes_only_rail_and_aim_joint() {
        let config = DynamixelConfig {
            stream_hz: 500.0,
            ..DynamixelConfig::default()
        };
        let mut hardware =
            RealHardware::dry_run(config, Some(test_rail())).expect("dry-run hardware");
        let before = hardware.read_pose().expect("before");

        let applied = hardware
            .command_rail_and_racket(0.35, -0.25, 0.1)
            .expect("tracking command");
        assert!((applied.rail_m - 0.35).abs() < 1e-9);
        assert!((applied.aim_rad - -0.25).abs() < 0.002);
        assert!(applied.rail_sent);

        // command_rail_and_racket이 반환하는 값은 클램프된 목표를 동기로
        // 계산한 것일 뿐, RailQueue 워커가 실제로 적용하는 건 비동기다.
        thread::sleep(Duration::from_millis(100));
        let after = hardware.read_pose().expect("after");
        assert!((after.rail_x - 0.35).abs() < 1e-9);
        for index in [0, 2, 3] {
            assert!((after.joints.values[index] - before.joints.values[index]).abs() < 0.002);
        }
        assert!((after.joints.values[1] - -0.25).abs() < 0.002);
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
