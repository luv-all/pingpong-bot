//! Dynamixel 4축 설정·좌표 변환과 Protocol 2.0 통신.
//!
//! SSOT: `test-manipulator`의 `DynamixelConfig` / `DynamixelController`.
//! - `radians_to_ticks` / `ticks_to_radians` = Python 동일 식
//! - Goal/Torque/Profile SyncWrite = Python `_pack_u32` / `_pack_u8`
//! - `enable_torque(true)` = profile 재적용 → (추가) Goal=Present → Torque ON

use std::f64::consts::TAU;

use thiserror::Error;

use crate::{HwError, Joints};

pub use crate::constants::dynamixel::{
    MX28_NO_LOAD_SPEED_RPM, MX28_STALL_TORQUE_NM, MX64_STALL_TORQUE_NM,
    PROFILE_VELOCITY_REV_MIN_PER_LSB, rev_min_to_rad_s,
};
pub use crate::defaults::dxl_limits::{
    CONTINUOUS_TORQUE_DERATE, DYNAMIXEL_MAX_JOINT_SPEED_RAD_S, JOINT_SPEED_DERATE,
    joint_torque_limits_4dof, joint_torque_limits_4dof_array,
};

/// Dynamixel Protocol 2.0 `Profile Velocity` 레지스터 값 → 관절 각속도 [rad/s].
///
/// 1 LSB = [`PROFILE_VELOCITY_REV_MIN_PER_LSB`] rev/min.
pub fn dynamixel_profile_velocity_to_rad_s(profile_velocity: u32) -> f64 {
    return rev_min_to_rad_s(f64::from(profile_velocity) * PROFILE_VELOCITY_REV_MIN_PER_LSB);
}

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

/// Goal Current (signed 16-bit) 패킹.
#[cfg(feature = "real")]
fn pack_i16(value: i16) -> Vec<u8> {
    value.to_le_bytes().to_vec()
}

/// 마스터 goal tick을 `2 * zero_tick - master`로 미러하는 슬레이브.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MirrorSlave {
    pub master_id: u8,
    pub slave_id: u8,
}

/// Dynamixel Protocol 2.0 버스 설정. 벤치 숫자는 `crate::defaults`에서 조립한다.
#[derive(Debug, Clone, PartialEq)]
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
    /// Protocol 2.0 Goal Current (MX-64 = 102).
    pub addr_goal_current: u8,
    /// Operating Mode 레지스터 (MX = 11).
    pub addr_operating_mode: u8,
    /// Current-based Position Control = 5.
    pub operating_mode_current_position: u8,
    /// Goal Current 1 unit → N·m (MX-64 ≈ 3.36 mA/unit, kt≈1.46 → ~0.0049).
    pub nm_per_goal_current_unit: f64,
    pub profile_acceleration: u32,
    pub profile_velocity: u32,
    pub comm_retries: u32,
    pub comm_retry_delay_ms: u64,
    pub stream_hz: f64,
    pub joint_signs: Vec<i8>,
    pub joint_offsets_rad: Vec<f64>,
    pub motor_angle_limits_deg: Vec<[f64; 2]>,
    pub mirror_slaves: Vec<MirrorSlave>,
}

/// Dynamixel 설정 검증 실패.
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
    #[error("mirror_slaves: master_id {master_id}가 motor_ids에 없습니다")]
    MirrorMasterMissing { master_id: u8 },
    #[error("mirror_slaves: slave_id {slave_id}는 motor_ids와 겹치면 안 됩니다")]
    MirrorSlaveInMotorIds { slave_id: u8 },
    #[error("mirror_slaves: id {id}가 중복됩니다")]
    MirrorDuplicateId { id: u8 },
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
        let mut seen_slaves = Vec::new();
        for pair in &self.mirror_slaves {
            if !self.motor_ids.contains(&pair.master_id) {
                return Err(DynamixelConfigError::MirrorMasterMissing {
                    master_id: pair.master_id,
                });
            }
            if self.motor_ids.contains(&pair.slave_id) {
                return Err(DynamixelConfigError::MirrorSlaveInMotorIds {
                    slave_id: pair.slave_id,
                });
            }
            if seen_slaves.contains(&pair.slave_id)
                || seen_slaves.contains(&pair.master_id)
                || pair.slave_id == pair.master_id
            {
                return Err(DynamixelConfigError::MirrorDuplicateId { id: pair.slave_id });
            }
            seen_slaves.push(pair.slave_id);
        }
        return Ok(());
    }

    /// Torque / Profile SyncWrite 대상 = 논리 모터 ∪ 미러 슬레이브.
    pub fn bus_ids(&self) -> Vec<u8> {
        let mut ids = self.motor_ids.clone();
        for pair in &self.mirror_slaves {
            if !ids.contains(&pair.slave_id) {
                ids.push(pair.slave_id);
            }
        }
        return ids;
    }

    pub fn mirror_tick(&self, master_ticks: i32) -> i32 {
        let mirrored = 2 * self.zero_tick - master_ticks;
        let max_tick = self.ticks_per_revolution.saturating_sub(1).max(0);
        return mirrored.clamp(0, max_tick);
    }
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
        /// `motor_ids` 순서 Present/Goal (읽기·논리 관절).
        ticks: Vec<i32>,
        /// 마지막 Goal SyncWrite 전체 (미러 슬레이브 포함).
        last_bus_goals: Vec<(u8, i32)>,
        /// 마지막 Goal Current (논리 모터 순서, signed units).
        last_goal_currents: Vec<i16>,
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
            backend: BusBackend::DryRun {
                ticks,
                last_bus_goals: Vec::new(),
                last_goal_currents: Vec::new(),
            },
            torque_enabled: false,
        });
    }

    /// dry-run 전용: 마지막 Goal에 실린 (id, tick), 미러 포함.
    pub fn last_bus_goals(&self) -> Option<&[(u8, i32)]> {
        return match &self.backend {
            BusBackend::DryRun { last_bus_goals, .. } => Some(last_bus_goals.as_slice()),
            #[cfg(feature = "real")]
            BusBackend::Real(_) => None,
        };
    }

    /// dry-run 전용: 마지막 Goal Current (논리 모터 순서).
    pub fn last_goal_currents(&self) -> Option<&[i16]> {
        return match &self.backend {
            BusBackend::DryRun {
                last_goal_currents, ..
            } => Some(last_goal_currents.as_slice()),
            #[cfg(feature = "real")]
            BusBackend::Real(_) => None,
        };
    }

    fn expand_goal_ticks(&self, ticks: &[i32]) -> Vec<(u8, i32)> {
        let cfg = &self.mapping.config;
        let mut out: Vec<(u8, i32)> = cfg
            .motor_ids
            .iter()
            .zip(ticks.iter())
            .map(|(&id, &tick)| (id, tick))
            .collect();
        for pair in &cfg.mirror_slaves {
            let Some(master_index) = cfg.motor_ids.iter().position(|&id| id == pair.master_id)
            else {
                continue;
            };
            out.push((pair.slave_id, cfg.mirror_tick(ticks[master_index])));
        }
        return out;
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
            .map_err(|error| {
                read_transport_error(format!(
                    "시리얼 포트 열기 실패 ({} @ {} baud): {error}",
                    mapping.config.port, mapping.config.baudrate
                ))
            })?;
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
                let ids = self.mapping.config.bus_ids();
                let data = vec![pack_u8(u8::from(enabled)); ids.len()];
                let address = self.mapping.config.addr_torque_enable;
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|error| {
                        read_transport_error(format!(
                            "Torque Enable sync_write 실패 (addr={address}): {error}"
                        ))
                    })?;
            }
        }
        self.torque_enabled = enabled;
        Ok(())
    }

    /// Python `set_joint_positions`.
    pub fn write_joints(&mut self, joints: &Joints) -> Result<(), HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        if joints.values.len() != joint_count {
            return Err(command_transport_error(
                0.0,
                joints.values.len(),
                format!("관절 수 불일치: got {} want {joint_count}", joints.values.len()),
            ));
        }
        let ticks: Vec<i32> = joints
            .values
            .iter()
            .enumerate()
            .map(|(index, angle)| self.mapping.radians_to_ticks(index, *angle))
            .collect();
        self.write_raw_goal_ticks(&ticks, 0.0)
    }

    /// RNEA \(\tau\) [N·m] → Goal Current SyncWrite (논리 모터 + 미러는 마스터와 동일 부호 전류).
    pub fn write_goal_currents_from_torques(&mut self, torques_nm: &[f64]) -> Result<(), HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        if torques_nm.len() != joint_count {
            return Err(command_transport_error(
                0.0,
                torques_nm.len(),
                format!(
                    "토크 벡터 길이 불일치: got {} want {joint_count}",
                    torques_nm.len()
                ),
            ));
        }
        let unit = self.mapping.config.nm_per_goal_current_unit.max(1e-9);
        let currents: Vec<i16> = torques_nm
            .iter()
            .map(|&tau| {
                let raw = (tau / unit).round();
                raw.clamp(i16::MIN as f64, i16::MAX as f64) as i16
            })
            .collect();
        return self.write_raw_goal_currents(&currents);
    }

    fn write_raw_goal_currents(&mut self, currents: &[i16]) -> Result<(), HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        if currents.len() != joint_count {
            return Err(command_transport_error(
                0.0,
                currents.len(),
                format!(
                    "Goal Current 길이 불일치: got {} want {joint_count}",
                    currents.len()
                ),
            ));
        }
        // 미러 슬레이브: 마스터와 같은 Goal Current (반대 방향 기구는 위치 미러로 처리).
        let mut bus: Vec<(u8, i16)> = self
            .mapping
            .config
            .motor_ids
            .iter()
            .zip(currents.iter())
            .map(|(&id, &c)| (id, c))
            .collect();
        for pair in &self.mapping.config.mirror_slaves {
            let Some(master_index) = self
                .mapping
                .config
                .motor_ids
                .iter()
                .position(|&id| id == pair.master_id)
            else {
                continue;
            };
            bus.push((pair.slave_id, currents[master_index]));
        }
        match &mut self.backend {
            BusBackend::DryRun {
                last_goal_currents, ..
            } => {
                *last_goal_currents = currents.to_vec();
            }
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids: Vec<u8> = bus.iter().map(|(id, _)| *id).collect();
                let data: Vec<Vec<u8>> = bus.iter().map(|(_, c)| pack_i16(*c)).collect();
                let address = self.mapping.config.addr_goal_current;
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|error| {
                        command_transport_error(
                            0.0,
                            joint_count,
                            format!("Goal Current sync_write 실패 (addr={address}): {error}"),
                        )
                    })?;
            }
        }
        return Ok(());
    }

    /// Torque OFF → Operating Mode = Current-based Position → (호출측에서 Torque ON).
    pub fn set_current_based_position_mode(&mut self) -> Result<(), HwError> {
        if self.torque_enabled {
            self.enable_torque(false)?;
        }
        match &mut self.backend {
            BusBackend::DryRun { .. } => {}
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let mode = self.mapping.config.operating_mode_current_position;
                let address = self.mapping.config.addr_operating_mode;
                let ids = self.mapping.config.bus_ids();
                let data: Vec<Vec<u8>> = ids.iter().map(|_| pack_u8(mode)).collect();
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|error| {
                        read_transport_error(format!(
                            "Operating Mode sync_write 실패 (addr={address}, mode={mode}): {error}"
                        ))
                    })?;
            }
        }
        return Ok(());
    }

    fn write_raw_goal_ticks(&mut self, ticks: &[i32], duration_secs: f64) -> Result<(), HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        if ticks.len() != joint_count {
            return Err(command_transport_error(
                duration_secs,
                ticks.len(),
                format!("Goal tick 길이 불일치: got {} want {joint_count}", ticks.len()),
            ));
        }
        let bus_goals = self.expand_goal_ticks(ticks);
        match &mut self.backend {
            BusBackend::DryRun {
                ticks: stored,
                last_bus_goals,
                ..
            } => {
                stored.clone_from_slice(ticks);
                *last_bus_goals = bus_goals;
            }
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids: Vec<u8> = bus_goals.iter().map(|(id, _)| *id).collect();
                let data: Vec<Vec<u8>> = bus_goals
                    .iter()
                    .map(|(_, tick)| pack_u32(*tick as u32))
                    .collect();
                let address = self.mapping.config.addr_goal_position;
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|error| {
                        command_transport_error(
                            duration_secs,
                            joint_count,
                            format!("Goal Position sync_write 실패 (addr={address}): {error}"),
                        )
                    })?;
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
            BusBackend::DryRun { ticks, .. } => ticks.clone(),
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
                .map_err(|error| {
                    read_transport_error(format!(
                        "Present Position sync_read 실패 (addr={}, ids={ids:?}): {error}",
                        self.mapping.config.addr_present_position
                    ))
                })?
                .into_iter()
                .map(|bytes| {
                    let raw: [u8; 4] = bytes.as_slice().try_into().map_err(|_| {
                        read_transport_error(format!(
                            "Present Position 응답 길이 오류: got {} bytes, want 4",
                            bytes.len()
                        ))
                    })?;
                    // Python SDK getData(4) → unsigned 해석 후 int. joint mode 0..=4095.
                    Ok(u32::from_le_bytes(raw) as i32)
                })
                .collect::<Result<Vec<_>, HwError>>()?
            }
        };
        if ticks.len() != joint_count {
            return Err(read_transport_error(format!(
                "Present Position 개수 불일치: got {} want {joint_count}",
                ticks.len()
            )));
        }
        Ok(ticks)
    }

    /// Python `apply_motion_profile` — Protocol 2.0 Profile Acc/Vel SyncWrite.
    fn apply_motion_profile(&mut self) -> Result<(), HwError> {
        match &mut self.backend {
            BusBackend::DryRun { .. } => {}
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids = self.mapping.config.bus_ids();
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
                        .map_err(|error| {
                            read_transport_error(format!(
                                "Motion Profile sync_write 실패 (addr={address}, value={value}): {error}"
                            ))
                        })?;
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
    ) -> Result<(), String> {
        self.run_with_retry(retries, retry_delay_ms, "sync_write", |protocol, port| {
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
    ) -> Result<Vec<Vec<u8>>, String> {
        self.run_with_retry(retries, retry_delay_ms, "sync_read", |protocol, port| {
            protocol.sync_read(port, ids, address, length)
        })
    }

    fn run_with_retry<T>(
        &mut self,
        retries: u32,
        retry_delay_ms: u64,
        op: &str,
        mut operation: impl FnMut(
            &rustypot::DynamixelProtocolHandler,
            &mut dyn serialport::SerialPort,
        ) -> Result<T, Box<dyn std::error::Error>>,
    ) -> Result<T, String> {
        let attempts = retries.max(1);
        let mut last_error = format!("{op}: no attempts");
        for attempt in 0..attempts {
            match operation(&self.protocol, self.port.as_mut()) {
                Ok(value) => return Ok(value),
                Err(error) if attempt + 1 < attempts => {
                    tracing::debug!(
                        op,
                        attempt = attempt + 1,
                        attempts,
                        error = %error,
                        "Dynamixel 통신 재시도"
                    );
                    last_error = error.to_string();
                    let _ = self.port.clear(serialport::ClearBuffer::All);
                    std::thread::sleep(std::time::Duration::from_millis(retry_delay_ms));
                }
                Err(error) => {
                    tracing::debug!(
                        op,
                        attempt = attempt + 1,
                        attempts,
                        error = %error,
                        "Dynamixel 통신 최종 실패"
                    );
                    return Err(error.to_string());
                }
            }
        }
        return Err(last_error);
    }
}

fn command_transport_error(
    duration_secs: f64,
    joint_count: usize,
    reason: impl Into<String>,
) -> HwError {
    return HwError::CommandFailed {
        duration_secs,
        joint_count,
        reason: reason.into(),
    };
}

fn read_transport_error(reason: impl Into<String>) -> HwError {
    return HwError::ReadFailed {
        reason: reason.into(),
    };
}

#[cfg(test)]
mod tests {
    use crate::Joints;

    use super::{
        DynamixelBus, DynamixelConfig, DynamixelConfigError, MotorMapping,
        dynamixel_profile_velocity_to_rad_s,
    };

    fn bench_config() -> DynamixelConfig {
        return DynamixelConfig::default();
    }

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
        let mapping = MotorMapping::new(bench_config()).expect("valid mapping");

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
        let mapping = MotorMapping::new(bench_config()).expect("valid mapping");

        let ticks = mapping.radians_to_ticks(2, -0.4);
        let restored = mapping.ticks_to_radians(2, ticks);
        assert!((restored - -0.4).abs() < 0.002);

        assert_eq!(mapping.radians_to_ticks(0, 100.0), 1024);
        assert_eq!(mapping.radians_to_ticks(0, -100.0), 2503);
    }

    #[test]
    fn motor_mapping_rejects_mismatched_vector_lengths() {
        let mut config = bench_config();
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
        let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
        let target = Joints::from_slice(&[-0.2, 0.1, -0.3, 0.2]);

        bus.enable_torque(true).expect("torque");
        bus.write_joints(&target).expect("write");
        let actual = bus.read_joints().expect("read");

        for (expected, actual) in target.values.iter().zip(actual.values) {
            assert!((expected - actual).abs() < 0.002);
        }
    }

    #[test]
    fn dry_run_mirrors_slave_goal_around_zero_tick() {
        let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
        // joint0 sign=-1 → URDF +angle decreases ticks from 2048.
        // Pick ticks via mapping: want master absolute ~2276 (200°) → slave 1820 (160°).
        let zero = bus.mapping.config().zero_tick;
        let ticks_per_rev = bus.mapping.config().ticks_per_revolution;
        let master_200 = (200.0 * f64::from(ticks_per_rev) / 360.0).round() as i32;
        let expected_slave = 2 * zero - master_200;

        // Drive joint0 so radians_to_ticks yields master_200 (within clamp).
        let angle = bus.mapping.ticks_to_radians(0, master_200);
        bus.write_joints(&Joints::from_slice(&[angle, 0.0, -0.26, 0.0]))
            .expect("write");
        let goals = bus.last_bus_goals().expect("dry-run goals");
        assert!(
            goals
                .iter()
                .any(|(id, tick)| *id == 1 && *tick == master_200)
        );
        assert!(
            goals
                .iter()
                .any(|(id, tick)| *id == 2 && *tick == expected_slave),
            "goals={goals:?} expected slave {expected_slave}"
        );
    }

    #[test]
    fn dry_run_goal_current_from_torque() {
        let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
        let unit = bus.mapping.config().nm_per_goal_current_unit;
        bus.write_goal_currents_from_torques(&[unit * 10.0, 0.0, -unit * 3.0, unit])
            .expect("currents");
        let currents = bus.last_goal_currents().expect("dry-run currents");
        assert_eq!(currents, &[10, 0, -3, 1]);
    }

    #[test]
    fn mirror_tick_formula() {
        let cfg = bench_config();
        assert_eq!(cfg.mirror_tick(2048), 2048);
        let t200 = (200.0_f64 * 4096.0 / 360.0).round() as i32;
        let t160 = (160.0_f64 * 4096.0 / 360.0).round() as i32;
        assert_eq!(cfg.mirror_tick(t200), t160);
    }
}
