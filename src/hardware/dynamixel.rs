//! Dynamixel 4축 설정·좌표 변환과 Protocol 2.0 통신.
//!
//! SSOT: `test-manipulator`의 `DynamixelConfig` / `DynamixelController`.
//! - `radians_to_ticks` / `ticks_to_radians` = Python 동일 식
//! - Goal/Torque/Profile SyncWrite = Python `_pack_u32` / `_pack_u8`
//! - `enable_torque(true)` = profile 재적용 → (추가) Goal=Present → Torque ON

use std::f64::consts::TAU;
use std::path::Path;

use serde::Deserialize;

use crate::{HwError, HwFailDetail, Joints};

/// Python `_pack_u32` — Goal Position / Profile 값 패킹.
fn pack_u32(value: u32) -> Vec<u8> {
    value.to_le_bytes().to_vec()
}

/// Python `_pack_u8` — Torque Enable 패킹.
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

impl DynamixelConfig {
    pub fn validate(&self) -> Result<(), String> {
        let joint_count = self.motor_ids.len();
        if joint_count != 4 {
            return Err(format!(
                "4-DOF RealHardware에는 motor_ids가 4개여야 합니다 (현재 {joint_count})"
            ));
        }
        for (name, len) in [
            ("joint_signs", self.joint_signs.len()),
            ("joint_offsets_rad", self.joint_offsets_rad.len()),
            ("motor_angle_limits_deg", self.motor_angle_limits_deg.len()),
        ] {
            if len != joint_count {
                return Err(format!("{name} 길이 {len} != motor_ids 길이 {joint_count}"));
            }
        }
        if self.joint_signs.iter().any(|sign| !matches!(sign, -1 | 1)) {
            return Err("joint_signs는 -1 또는 1이어야 합니다".to_owned());
        }
        if self.ticks_per_revolution <= 0 {
            return Err("ticks_per_revolution은 0보다 커야 합니다".to_owned());
        }
        if self.protocol_version != 2.0 {
            return Err("현재 RealHardware는 Dynamixel Protocol 2.0만 지원합니다".to_owned());
        }
        if !self.stream_hz.is_finite() || self.stream_hz <= 0.0 {
            return Err("stream_hz는 0보다 커야 합니다".to_owned());
        }
        if self
            .motor_angle_limits_deg
            .iter()
            .any(|[lo, hi]| !lo.is_finite() || !hi.is_finite() || lo > hi)
        {
            return Err("motor_angle_limits_deg 범위가 잘못됐습니다".to_owned());
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
pub fn config_from_toml(text: &str) -> Result<DynamixelConfig, String> {
    let document: RuntimeHardwareDocument =
        toml::from_str(text).map_err(|error| error.to_string())?;
    document.hardware.dynamixel.validate()?;
    return Ok(document.hardware.dynamixel);
}

pub fn load_config(path: &Path) -> Result<DynamixelConfig, String> {
    let text = std::fs::read_to_string(path).map_err(|error| error.to_string())?;
    return config_from_toml(&text);
}

/// URDF 관절각과 Dynamixel 절대 tick 사이의 순수 좌표 변환.
#[derive(Debug, Clone)]
pub struct MotorMapping {
    config: DynamixelConfig,
    tick_limits: Vec<(i32, i32)>,
}

impl MotorMapping {
    pub fn new(config: DynamixelConfig) -> Result<Self, String> {
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
    pub fn dry_run(config: DynamixelConfig) -> Result<Self, String> {
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
        let mapping = MotorMapping::new(config).map_err(|_| read_transport_error())?;
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
        let ids = self.mapping.config.motor_ids.clone();
        let data = vec![pack_u8(u8::from(enabled)); ids.len()];
        let address = self.mapping.config.addr_torque_enable;
        let retries = self.mapping.config.comm_retries;
        let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
        match &mut self.backend {
            BusBackend::DryRun { .. } => {}
            #[cfg(feature = "real")]
            BusBackend::Real(real) => real
                .sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                .map_err(|_| read_transport_error())?,
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
        let data: Vec<Vec<u8>> = ticks.iter().map(|tick| pack_u32(*tick as u32)).collect();
        let ids = self.mapping.config.motor_ids.clone();
        let address = self.mapping.config.addr_goal_position;
        let retries = self.mapping.config.comm_retries;
        let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
        match &mut self.backend {
            BusBackend::DryRun { ticks: stored } => stored.clone_from_slice(ticks),
            #[cfg(feature = "real")]
            BusBackend::Real(real) => real
                .sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                .map_err(|_| command_transport_error(duration_secs, joint_count))?,
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
        match &mut self.backend {
            BusBackend::DryRun { .. } => {}
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
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
        detail: HwFailDetail::Transport,
    };
}

fn read_transport_error() -> HwError {
    return HwError::ReadFailed {
        detail: HwFailDetail::Transport,
    };
}

#[cfg(test)]
mod tests {
    use crate::Joints;

    use super::{DynamixelBus, DynamixelConfig, MotorMapping, config_from_toml};

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
        assert!(error.contains("joint_signs"));
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
