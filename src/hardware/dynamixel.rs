//! Dynamixel 4축 설정·좌표 변환과 Protocol 2.0 통신.
//!
//! SSOT: `test-manipulator`의 `DynamixelConfig` / `DynamixelController`.
//! - `radians_to_ticks` / `ticks_to_radians` = Python 동일 식
//! - Goal/Torque/Profile SyncWrite = Python `_pack_u32` / `_pack_u8`
//! - `enable_torque(true)` = profile 재적용 → (추가) Goal=Present → Torque ON

use std::f64::consts::TAU;
use std::path::Path;

use serde::Deserialize;
use thiserror::Error;

use crate::{HwError, Joints};

/// Dynamixel Protocol 2.0 `Profile Velocity` 레지스터 단위 (velocity-based profile mode).
///
/// source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ 와
/// https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ (Protocol 2.0 control table,
/// address 112 "Profile Velocity"), retrieved 2026-07-23. MX-64/MX-28 동일.
/// 자세한 조사 근거: `.omc/research/dynamixel-specs.md`.
pub const PROFILE_VELOCITY_REV_MIN_PER_LSB: f64 = 0.229;

/// MX-28T 무부하 속도 [rev/min], 12.0V (Robotis 권장 동작 전압).
///
/// source: https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/, retrieved 2026-07-23.
/// 4축 체인에서 팔꿈치·손목을 구동하는 모터(`assets/robots/4-dof/README.md` 매핑).
/// 어깨·yaw를 구동하는 MX-64R(63 rpm@12V)보다 느려서, 단일 스칼라 `max_joint_speed`의
/// 기준을 이 쪽으로 잡는다 — 실제 로봇에서 팔 전체는 가장 느린 관절 속도로 제약된다.
/// 실기 버스 전압은 이 repo 어디에도 명시돼 있지 않아 Robotis 권장값(12V)을 쓴다.
const MX28_NO_LOAD_SPEED_RPM: f64 = 55.0;

/// rev/min -> rad/s.
const fn rev_min_to_rad_s(rev_min: f64) -> f64 {
    return rev_min * TAU / 60.0;
}

/// Dynamixel Protocol 2.0 `Profile Velocity` 레지스터 값(velocity-based profile mode) ->
/// 관절 각속도 [rad/s].
///
/// 1 LSB = 0.229 rev/min ([`PROFILE_VELOCITY_REV_MIN_PER_LSB`]). `config/real-hardware.toml`의
/// `profile_velocity`가 이 단위. 예: `profile_velocity = 80` -> 80 * 0.229 = 18.32 rev/min
/// -> 18.32 * 2*PI/60 ≈ 1.918 rad/s.
pub fn dynamixel_profile_velocity_to_rad_s(profile_velocity: u32) -> f64 {
    return rev_min_to_rad_s(f64::from(profile_velocity) * PROFILE_VELOCITY_REV_MIN_PER_LSB);
}

/// 실기(Dynamixel 4축) 물리 스펙 기반 관절 속도 상한 [rad/s] — **SSOT**.
///
/// `Arm::competition()`(`src/robot/mod.rs`)과 URDF 카탈로그(`urdf-test`, `4-dof`,
/// `src/robot/catalog.rs`) 모두 같은 물리 로봇을 모델링하므로 (competition의 타입
/// 문서 참고) 이 상수 하나를 공유한다. 예전에는 이 둘이 각각 `16.0`
/// (`constants::arm::MAX_JOINT_SPEED`, 근거 없는 리터럴)과 `2.5`(placeholder)로
/// 따로 놀았다 — 둘 다 삭제하고 이 값으로 통일.
///
/// `config/real-hardware.toml`의 `profile_velocity = 80` (-> `dynamixel_profile_velocity_to_rad_s(80)`
/// ≈ 1.92 rad/s)이 아니라 [`MX28_NO_LOAD_SPEED_RPM`]을 기준으로 계산한다: 그 config 값은
/// 파일 자체 주석("conservative 조그·검증용")에 물리적 상한이 아니라고 명시돼 있어,
/// sim 궤적 계획(특히 이 브랜치의 rough-to-fine 스윙 다이나믹스)에 그대로 박아 넣으면
/// 실기의 실제 스윙 능력을 과소평가하게 된다. 대신 무부하 속도에서 50% derate한다 —
/// 부하(팔+라켓 질량, 공기저항) 아래 지속 구동 시 무부하 대비 대략적인 안전 마진으로,
/// Robotis 공식 수치가 아닌 엔지니어링 판단이다.
pub const DYNAMIXEL_MAX_JOINT_SPEED_RAD_S: f64 = rev_min_to_rad_s(MX28_NO_LOAD_SPEED_RPM) * 0.5;

/// Python `_pack_u32` — Goal Position / Profile 값 패킹.
#[cfg(feature = "real")]
fn pack_u32(value: u32) -> Vec<u8> {
    value.to_le_bytes().to_vec()
}

/// Python `_pack_u8` — Torque Enable 패킹.
#[cfg(feature = "real")]
fn pack_u8(value: u8) -> Vec<u8> {
    vec![value]
}

/// Python `test-manipulator`에서 검증한 Dynamixel Protocol 2.0 설정.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default)]
pub struct DynamixelConfig {
    pub port: String,
    pub baudrate: u32,
    pub protocol_version: f32,
    pub motor_ids: Vec<u8>,
    pub ticks_per_revolution: i32,
    pub zero_tick: i32,
    pub addr_goal_position: u8,
    pub addr_torque_enable: u8,
    pub addr_present_position: u8,
    pub addr_profile_acceleration: u8,
    pub addr_profile_velocity: u8,
    pub profile_acceleration: u32,
    pub profile_velocity: u32,
    pub comm_retries: u32,
    pub comm_retry_delay_ms: u64,
    pub stream_hz: f64,
    pub joint_signs: Vec<i8>,
    pub joint_offsets_rad: Vec<f64>,
    pub motor_angle_limits_deg: Vec<[f64; 2]>,
}

impl Default for DynamixelConfig {
    fn default() -> Self {
        return Self {
            port: "COM8".to_owned(),
            baudrate: 57_600,
            protocol_version: 2.0,
            motor_ids: vec![1, 3, 4, 5],
            ticks_per_revolution: 4096,
            zero_tick: 2048,
            addr_goal_position: 116,
            addr_torque_enable: 64,
            addr_present_position: 132,
            addr_profile_acceleration: 108,
            addr_profile_velocity: 112,
            profile_acceleration: 20,
            profile_velocity: 80,
            comm_retries: 5,
            comm_retry_delay_ms: 20,
            stream_hz: 200.0,
            joint_signs: vec![-1, 1, 1, 1],
            joint_offsets_rad: vec![0.0; 4],
            motor_angle_limits_deg: vec![
                [90.0, 220.0],
                [135.0, 225.0],
                [92.0, 230.0],
                [120.0, 220.0],
            ],
        };
    }
}

/// Dynamixel TOML/설정 검증·로드 실패.
#[derive(Debug, Error)]
pub enum DynamixelConfigError {
    #[error("4-DOF RealHardware에는 motor_ids가 4개여야 합니다 (현재 {joint_count})")]
    MotorCount { joint_count: usize },
    #[error("{name} 길이 {len} != motor_ids 길이 {joint_count}")]
    VectorLength {
        name: &'static str,
        len: usize,
        joint_count: usize,
    },
    #[error("joint_signs는 -1 또는 1이어야 합니다")]
    JointSigns,
    #[error("ticks_per_revolution은 0보다 커야 합니다")]
    TicksPerRevolution,
    #[error("현재 RealHardware는 Dynamixel Protocol 2.0만 지원합니다")]
    ProtocolVersion,
    #[error("stream_hz는 0보다 커야 합니다")]
    StreamHz,
    #[error("motor_angle_limits_deg 범위가 잘못됐습니다")]
    AngleLimits,
    #[error("TOML 파싱 실패: {0}")]
    Toml(#[from] toml::de::Error),
    #[error("설정 파일 읽기 실패: {0}")]
    Io(#[from] std::io::Error),
}

impl DynamixelConfig {
    pub fn validate(&self) -> Result<(), DynamixelConfigError> {
        let joint_count = self.motor_ids.len();
        if joint_count != 4 {
            return Err(DynamixelConfigError::MotorCount { joint_count });
        }
        for (name, len) in [
            ("joint_signs", self.joint_signs.len()),
            ("joint_offsets_rad", self.joint_offsets_rad.len()),
            ("motor_angle_limits_deg", self.motor_angle_limits_deg.len()),
        ] {
            if len != joint_count {
                return Err(DynamixelConfigError::VectorLength {
                    name,
                    len,
                    joint_count,
                });
            }
        }
        if self.joint_signs.iter().any(|sign| !matches!(sign, -1 | 1)) {
            return Err(DynamixelConfigError::JointSigns);
        }
        if self.ticks_per_revolution <= 0 {
            return Err(DynamixelConfigError::TicksPerRevolution);
        }
        if self.protocol_version != 2.0 {
            return Err(DynamixelConfigError::ProtocolVersion);
        }
        if !self.stream_hz.is_finite() || self.stream_hz <= 0.0 {
            return Err(DynamixelConfigError::StreamHz);
        }
        if self
            .motor_angle_limits_deg
            .iter()
            .any(|[lo, hi]| !lo.is_finite() || !hi.is_finite() || lo > hi)
        {
            return Err(DynamixelConfigError::AngleLimits);
        }
        return Ok(());
    }
}

#[derive(Deserialize)]
struct RuntimeHardwareDocument {
    hardware: RuntimeHardwareSection,
}

#[derive(Deserialize)]
struct RuntimeHardwareSection {
    dynamixel: DynamixelConfig,
}

/// 전체 런타임 TOML에서 `[hardware.dynamixel]`만 읽는다.
pub fn config_from_toml(text: &str) -> Result<DynamixelConfig, DynamixelConfigError> {
    let document: RuntimeHardwareDocument = toml::from_str(text)?;
    document.hardware.dynamixel.validate()?;
    return Ok(document.hardware.dynamixel);
}

pub fn load_config(path: &Path) -> Result<DynamixelConfig, DynamixelConfigError> {
    let text = std::fs::read_to_string(path)?;
    return config_from_toml(&text);
}

/// URDF 관절각과 Dynamixel 절대 tick 사이의 순수 좌표 변환.
#[derive(Debug, Clone)]
pub struct MotorMapping {
    config: DynamixelConfig,
    tick_limits: Vec<(i32, i32)>,
}

impl MotorMapping {
    pub fn new(config: DynamixelConfig) -> Result<Self, DynamixelConfigError> {
        config.validate()?;
        let tick_limits = config
            .motor_angle_limits_deg
            .iter()
            .map(|[lo, hi]| {
                let to_tick = |degrees: f64| {
                    (degrees * f64::from(config.ticks_per_revolution) / 360.0).round() as i32
                };
                (to_tick(*lo), to_tick(*hi))
            })
            .collect();
        return Ok(Self {
            config,
            tick_limits,
        });
    }

    pub fn config(&self) -> &DynamixelConfig {
        return &self.config;
    }

    pub fn radians_to_ticks(&self, joint_index: usize, angle_rad: f64) -> i32 {
        let sign = f64::from(self.config.joint_signs[joint_index]);
        let adjusted = sign * angle_rad + self.config.joint_offsets_rad[joint_index];
        let ticks = (f64::from(self.config.zero_tick)
            + adjusted * f64::from(self.config.ticks_per_revolution) / TAU)
            .round() as i32;
        let (lo, hi) = self.tick_limits[joint_index];
        return ticks.clamp(lo, hi);
    }

    pub fn ticks_to_radians(&self, joint_index: usize, ticks: i32) -> f64 {
        let raw = f64::from(ticks - self.config.zero_tick) * TAU
            / f64::from(self.config.ticks_per_revolution);
        let sign = f64::from(self.config.joint_signs[joint_index]);
        return sign * (raw - self.config.joint_offsets_rad[joint_index]);
    }
}

enum BusBackend {
    DryRun {
        ticks: Vec<i32>,
    },
    #[cfg(feature = "real")]
    Real(RealBackend),
}

/// Protocol 2.0 버스. dry-run도 같은 좌표 변환·리밋 경로를 사용한다.
pub struct DynamixelBus {
    mapping: MotorMapping,
    backend: BusBackend,
    torque_enabled: bool,
}

impl DynamixelBus {
    pub fn dry_run(config: DynamixelConfig) -> Result<Self, DynamixelConfigError> {
        let mapping = MotorMapping::new(config)?;
        let ticks = (0..mapping.config.motor_ids.len())
            .map(|index| mapping.radians_to_ticks(index, 0.0))
            .collect();
        return Ok(Self {
            mapping,
            backend: BusBackend::DryRun { ticks },
            torque_enabled: false,
        });
    }

    #[cfg(feature = "real")]
    pub fn open(config: DynamixelConfig) -> Result<Self, HwError> {
        let mapping = MotorMapping::new(config).map_err(|e| HwError::InvalidConfig {
            reason: e.to_string(),
        })?;
        let timeout = std::time::Duration::from_millis(100);
        let port = serialport::new(&mapping.config.port, mapping.config.baudrate)
            .timeout(timeout)
            .open()
            .map_err(|_| read_transport_error())?;
        let mut bus = Self {
            mapping,
            backend: BusBackend::Real(RealBackend {
                protocol: rustypot::DynamixelProtocolHandler::v2(),
                port,
            }),
            torque_enabled: false,
        };
        bus.apply_motion_profile()?;
        return Ok(bus);
    }

    /// Python `enable_torque`: Torque ON이면 profile 재적용 후 Torque Enable SyncWrite.
    ///
    /// Rust 추가 안전: Torque ON 직전 Present를 Goal에 맞춰 잔여 Goal 급기동을 막는다.
    pub fn enable_torque(&mut self, enabled: bool) -> Result<(), HwError> {
        if enabled {
            self.apply_motion_profile()?;
            let present = self.read_raw_ticks()?;
            self.write_raw_goal_ticks(&present, 0.0)?;
        }
        match &mut self.backend {
            BusBackend::DryRun { .. } => {}
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids = self.mapping.config.motor_ids.clone();
                let data = vec![pack_u8(u8::from(enabled)); ids.len()];
                let address = self.mapping.config.addr_torque_enable;
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|_| read_transport_error())?;
            }
        }
        self.torque_enabled = enabled;
        Ok(())
    }

    /// Python `set_joint_positions`.
    pub fn write_joints(&mut self, joints: &Joints) -> Result<(), HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        if joints.values.len() != joint_count {
            return Err(command_transport_error(0.0, joints.values.len()));
        }
        let ticks: Vec<i32> = joints
            .values
            .iter()
            .enumerate()
            .map(|(index, angle)| self.mapping.radians_to_ticks(index, *angle))
            .collect();
        self.write_raw_goal_ticks(&ticks, 0.0)
    }

    fn write_raw_goal_ticks(&mut self, ticks: &[i32], duration_secs: f64) -> Result<(), HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        if ticks.len() != joint_count {
            return Err(command_transport_error(duration_secs, ticks.len()));
        }
        // Python `_pack_u32` — Goal Position은 unsigned LE 4바이트.
        match &mut self.backend {
            BusBackend::DryRun { ticks: stored } => stored.clone_from_slice(ticks),
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let data: Vec<Vec<u8>> = ticks.iter().map(|tick| pack_u32(*tick as u32)).collect();
                let ids = self.mapping.config.motor_ids.clone();
                let address = self.mapping.config.addr_goal_position;
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|_| command_transport_error(duration_secs, joint_count))?;
            }
        }
        Ok(())
    }

    /// Python `read_joint_positions`.
    pub fn read_joints(&mut self) -> Result<Joints, HwError> {
        let ticks = self.read_raw_ticks()?;
        Ok(Joints {
            values: ticks
                .into_iter()
                .enumerate()
                .map(|(index, tick)| self.mapping.ticks_to_radians(index, tick))
                .collect(),
        })
    }

    fn read_raw_ticks(&mut self) -> Result<Vec<i32>, HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        let ticks = match &mut self.backend {
            BusBackend::DryRun { ticks } => ticks.clone(),
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids = self.mapping.config.motor_ids.clone();
                real.sync_read_with_retry(
                    &ids,
                    self.mapping.config.addr_present_position,
                    4,
                    self.mapping.config.comm_retries,
                    self.mapping.config.comm_retry_delay_ms,
                )
                .map_err(|_| read_transport_error())?
                .into_iter()
                .map(|bytes| {
                    let raw: [u8; 4] = bytes
                        .as_slice()
                        .try_into()
                        .map_err(|_| read_transport_error())?;
                    // Python SDK getData(4) → unsigned 해석 후 int. joint mode 0..=4095.
                    Ok(u32::from_le_bytes(raw) as i32)
                })
                .collect::<Result<Vec<_>, HwError>>()?
            }
        };
        if ticks.len() != joint_count {
            return Err(read_transport_error());
        }
        Ok(ticks)
    }

    /// Python `apply_motion_profile` — Protocol 2.0 Profile Acc/Vel SyncWrite.
    fn apply_motion_profile(&mut self) -> Result<(), HwError> {
        match &mut self.backend {
            BusBackend::DryRun { .. } => {}
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids = self.mapping.config.motor_ids.clone();
                let retries = self.mapping.config.comm_retries;
                let delay = self.mapping.config.comm_retry_delay_ms;
                let values = [
                    (
                        self.mapping.config.addr_profile_acceleration,
                        self.mapping.config.profile_acceleration,
                    ),
                    (
                        self.mapping.config.addr_profile_velocity,
                        self.mapping.config.profile_velocity,
                    ),
                ];
                for (address, value) in values {
                    let data = vec![pack_u32(value); ids.len()];
                    real.sync_write_with_retry(&ids, address, &data, retries, delay)
                        .map_err(|_| read_transport_error())?;
                }
            }
        }
        Ok(())
    }
}

impl Drop for DynamixelBus {
    fn drop(&mut self) {
        // Python `close`: best-effort torque off (+ port Drop이 시리얼 닫음).
        if self.torque_enabled {
            let _ = self.enable_torque(false);
        }
    }
}

#[cfg(feature = "real")]
struct RealBackend {
    protocol: rustypot::DynamixelProtocolHandler,
    port: Box<dyn serialport::SerialPort>,
}

#[cfg(feature = "real")]
impl RealBackend {
    fn sync_write_with_retry(
        &mut self,
        ids: &[u8],
        address: u8,
        data: &[Vec<u8>],
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<(), ()> {
        self.run_with_retry(retries, retry_delay_ms, |protocol, port| {
            protocol.sync_write(port, ids, address, data)
        })
    }

    fn sync_read_with_retry(
        &mut self,
        ids: &[u8],
        address: u8,
        length: u8,
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<Vec<Vec<u8>>, ()> {
        self.run_with_retry(retries, retry_delay_ms, |protocol, port| {
            protocol.sync_read(port, ids, address, length)
        })
    }

    fn run_with_retry<T>(
        &mut self,
        retries: u32,
        retry_delay_ms: u64,
        mut operation: impl FnMut(
            &rustypot::DynamixelProtocolHandler,
            &mut dyn serialport::SerialPort,
        ) -> Result<T, Box<dyn std::error::Error>>,
    ) -> Result<T, ()> {
        let attempts = retries.max(1);
        for attempt in 0..attempts {
            match operation(&self.protocol, self.port.as_mut()) {
                Ok(value) => return Ok(value),
                Err(_) if attempt + 1 < attempts => {
                    let _ = self.port.clear(serialport::ClearBuffer::All);
                    std::thread::sleep(std::time::Duration::from_millis(retry_delay_ms));
                }
                Err(_) => return Err(()),
            }
        }
        return Err(());
    }
}

fn command_transport_error(duration_secs: f64, joint_count: usize) -> HwError {
    return HwError::CommandFailed {
        duration_secs,
        joint_count,
    };
}

fn read_transport_error() -> HwError {
    return HwError::ReadFailed;
}

#[cfg(test)]
mod tests {
    use crate::Joints;

    use super::{
        DynamixelBus, DynamixelConfig, DynamixelConfigError, MotorMapping, config_from_toml,
        dynamixel_profile_velocity_to_rad_s,
    };

    #[test]
    fn profile_velocity_to_rad_s_matches_hand_computed_value() {
        // config/real-hardware.toml의 profile_velocity = 80.
        // 80 LSB * 0.229 rev/min/LSB = 18.32 rev/min
        // 18.32 rev/min * 2*PI rad/rev / 60 s/min ≈ 1.918466 rad/s
        // source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ (Profile Velocity
        // unit, Protocol 2.0 control table, addr 112), retrieved 2026-07-23.
        let rad_s = dynamixel_profile_velocity_to_rad_s(80);
        assert!((rad_s - 1.918_466).abs() < 1e-4);

        // 0 LSB -> 0 rad/s (register `0` also means "infinite velocity" on real hardware,
        // but the pure unit conversion itself must still be 0).
        assert!((dynamixel_profile_velocity_to_rad_s(0) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn motor_mapping_matches_python_reference() {
        let mapping = MotorMapping::new(DynamixelConfig::default()).expect("valid mapping");

        assert_eq!(mapping.radians_to_ticks(0, 0.0), 2048);
        assert_eq!(
            mapping.radians_to_ticks(0, std::f64::consts::FRAC_PI_2),
            1024
        );
        assert_eq!(
            mapping.radians_to_ticks(1, std::f64::consts::FRAC_PI_2),
            2560
        );
    }

    #[test]
    fn motor_mapping_round_trips_and_clamps_to_motor_limits() {
        let mapping = MotorMapping::new(DynamixelConfig::default()).expect("valid mapping");

        let ticks = mapping.radians_to_ticks(2, -0.4);
        let restored = mapping.ticks_to_radians(2, ticks);
        assert!((restored - -0.4).abs() < 0.002);

        assert_eq!(mapping.radians_to_ticks(0, 100.0), 1024);
        assert_eq!(mapping.radians_to_ticks(0, -100.0), 2503);
    }

    #[test]
    fn motor_mapping_rejects_mismatched_vector_lengths() {
        let mut config = DynamixelConfig::default();
        config.joint_signs.pop();

        let error = MotorMapping::new(config).unwrap_err();
        assert!(matches!(
            error,
            DynamixelConfigError::VectorLength {
                name: "joint_signs",
                ..
            }
        ));
    }

    #[test]
    fn dry_run_bus_round_trips_last_written_joints() {
        let mut bus = DynamixelBus::dry_run(DynamixelConfig::default()).expect("dry-run bus");
        let target = Joints::from_slice(&[-0.2, 0.1, -0.3, 0.2]);

        bus.enable_torque(true).expect("torque");
        bus.write_joints(&target).expect("write");
        let actual = bus.read_joints().expect("read");

        for (expected, actual) in target.values.iter().zip(actual.values) {
            assert!((expected - actual).abs() < 0.002);
        }
    }

    #[test]
    fn reads_dynamixel_section_from_runtime_toml() {
        let config = config_from_toml(
            r#"
[hardware.dynamixel]
port = "COM9"
motor_ids = [1, 3, 4, 5]
"#,
        )
        .expect("config");

        assert_eq!(config.port, "COM9");
        assert_eq!(config.baudrate, 57_600);
    }
}
