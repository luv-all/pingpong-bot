use crate::error::HwError;
use crate::robot::Joints;

use super::bus_backend::BusBackend;
use super::dynamixel_config::DynamixelConfig;
use super::dynamixel_config_error::DynamixelConfigError;
use super::motor_mapping::MotorMapping;
#[cfg(feature = "real")]
use super::real_backend::RealBackend;

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

/// Protocol 2.0 버스. dry-run도 같은 좌표 변환·리밋 경로를 사용한다.
pub struct DynamixelBus {
    pub(super) mapping: MotorMapping,
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
            backend: BusBackend::Real(RealBackend::new(
                rustypot::DynamixelProtocolHandler::v2(),
                port,
            )),
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
                format!(
                    "관절 수 불일치: got {} want {joint_count}",
                    joints.values.len()
                ),
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
                format!(
                    "Goal tick 길이 불일치: got {} want {joint_count}",
                    ticks.len()
                ),
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
