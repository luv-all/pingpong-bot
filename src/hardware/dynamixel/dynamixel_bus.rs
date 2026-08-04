use tracing::{debug, warn};

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

/// PWM Limit / Current Limit (unsigned 16-bit) 패킹.
#[cfg(feature = "real")]
fn pack_u16(value: u16) -> Vec<u8> {
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
    /// 클램프 경고 스로틀 — 200 Hz 스트리밍이라 매번 찍으면 로그가 잠긴다.
    last_clamp_warn: Option<std::time::Instant>,
}

/// 모터 한계 클램프 경고 주기.
const CLAMP_WARN_PERIOD: std::time::Duration = std::time::Duration::from_secs(1);

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
                last_operating_mode: None,
                last_pwm_limits: Vec::new(),
                last_current_limits: Vec::new(),
            },
            torque_enabled: false,
            last_clamp_warn: None,
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

    /// dry-run 전용: 마지막 Operating Mode.
    pub fn last_operating_mode(&self) -> Option<u8> {
        return match &self.backend {
            BusBackend::DryRun {
                last_operating_mode,
                ..
            } => *last_operating_mode,
            #[cfg(feature = "real")]
            BusBackend::Real(_) => None,
        };
    }

    /// dry-run 전용: 마지막 PWM Limit (버스 ID, 값).
    pub fn last_pwm_limits(&self) -> Option<&[(u8, u16)]> {
        return match &self.backend {
            BusBackend::DryRun {
                last_pwm_limits, ..
            } => Some(last_pwm_limits.as_slice()),
            #[cfg(feature = "real")]
            BusBackend::Real(_) => None,
        };
    }

    /// dry-run 전용: 마지막 Current Limit (MX-64 ID만).
    pub fn last_current_limits(&self) -> Option<&[(u8, u16)]> {
        return match &self.backend {
            BusBackend::DryRun {
                last_current_limits,
                ..
            } => Some(last_current_limits.as_slice()),
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
            last_clamp_warn: None,
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
    ///
    /// 모터 각도 한계에 걸리는 관절이 있으면 경고한다 — 플래너는 URDF 한계만 보므로
    /// `motor_angle_limits_deg`가 더 좁으면 여기서 조용히 잘려 팔이 명령과 다른 곳에 선다.
    /// 200 Hz 스트리밍이라 스로틀한다.
    pub fn write_joints(&mut self, joints: &Joints) -> Result<(), HwError> {
        self.write_joints_applied(joints).map(|_| ())
    }

    /// 전체 관절 Goal을 보내고 모터 한계 clamp·tick 양자화가 반영된 각도를 반환한다.
    pub fn write_joints_applied(&mut self, joints: &Joints) -> Result<Joints, HwError> {
        let clamped: Vec<usize> = joints
            .values
            .iter()
            .enumerate()
            .filter(|(index, angle)| self.mapping.clamped_by_motor_limit(*index, **angle))
            .map(|(index, _)| index)
            .collect();
        if !clamped.is_empty()
            && self
                .last_clamp_warn
                .is_none_or(|at| at.elapsed() >= CLAMP_WARN_PERIOD)
        {
            self.last_clamp_warn = Some(std::time::Instant::now());
            warn!(
                joints = ?clamped,
                commanded = ?joints.values,
                "모터 각도 한계로 잘림 — 계획한 자세에 못 선다 (플래너는 URDF 한계만 본다)"
            );
        }
        let applied = self.quantize_joints(joints)?;
        let ticks: Vec<i32> = applied
            .values
            .iter()
            .enumerate()
            .map(|(index, angle)| self.mapping.radians_to_ticks(index, *angle))
            .collect();
        self.write_raw_goal_ticks(&ticks, 0.0)?;
        return Ok(applied);
    }

    /// 버스에 쓰지 않고 모터 한계 clamp·tick 양자화 결과만 계산한다.
    pub fn quantize_joints(&self, joints: &Joints) -> Result<Joints, HwError> {
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
        return Ok(Joints {
            values: ticks
                .iter()
                .enumerate()
                .map(|(index, tick)| self.mapping.ticks_to_radians(index, *tick))
                .collect(),
        });
    }

    /// 한 관절의 Goal Position만 보낸다.
    ///
    /// 2단계 제어 시험에서는 팔 전체를 다시 명령하지 않고 라켓을 잡은 마지막
    /// 관절만 움직여야 한다. 전체 SyncWrite를 재사용하면 나머지 관절도 매 프레임
    /// 제어 대상이 되므로 별도 단일축 경로를 둔다.
    pub fn write_joint(&mut self, joint_index: usize, angle_rad: f64) -> Result<f64, HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        let Some(&motor_id) = self.mapping.config.motor_ids.get(joint_index) else {
            return Err(command_transport_error(
                0.0,
                1,
                format!("관절 인덱스 범위 초과: got {joint_index} want < {joint_count}"),
            ));
        };
        if self.mapping.clamped_by_motor_limit(joint_index, angle_rad)
            && self
                .last_clamp_warn
                .is_none_or(|at| at.elapsed() >= CLAMP_WARN_PERIOD)
        {
            self.last_clamp_warn = Some(std::time::Instant::now());
            warn!(
                joint = joint_index,
                commanded = angle_rad,
                "단일 관절 각도 한계로 잘림"
            );
        }

        let tick = self.mapping.radians_to_ticks(joint_index, angle_rad);
        let mut bus_goals = vec![(motor_id, tick)];
        for pair in &self.mapping.config.mirror_slaves {
            if pair.master_id == motor_id {
                bus_goals.push((pair.slave_id, self.mapping.config.mirror_tick(tick)));
            }
        }
        match &mut self.backend {
            BusBackend::DryRun {
                ticks,
                last_bus_goals,
                ..
            } => {
                ticks[joint_index] = tick;
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
                real.sync_write_with_retry(
                    &ids,
                    address,
                    &data,
                    self.mapping.config.comm_retries,
                    self.mapping.config.comm_retry_delay_ms,
                )
                .map_err(|error| {
                    command_transport_error(
                        0.0,
                        1,
                        format!("단일축 Goal Position sync_write 실패 (addr={address}): {error}"),
                    )
                })?;
            }
        }
        return Ok(self.mapping.ticks_to_radians(joint_index, tick));
    }

    /// Position Control + PWM/Current Limit 최대 → (호출측에서 Torque ON).
    ///
    /// Position 모드(3)에서는 Goal Current가 쓰이지 않고, 출력 상한은 PWM Limit이다.
    /// MX-64 Current Limit도 스펙 최대로 맞춰 둔다(MX-28에는 해당 레지스터 없음).
    ///
    /// 세 레지스터(11 · 36 · 38)는 전부 **EEPROM**(addr 0~63)이라 Torque Enable=1이면 쓰기가
    /// 거부된다. 그래서 먼저 읽어보고 **이미 원하는 값이면 아무것도 쓰지 않는다** — 앞 실행이
    /// 토크를 켠 채 끝냈어도([`Self::hold_torque_on_close`]) 팔이 잠깐 늘어지는 일이 없다.
    /// 값이 다를 때만 토크를 내리고 쓴다.
    pub fn configure_position_mode_max_effort(&mut self) -> Result<(), HwError> {
        if self.position_mode_already_configured()? {
            debug!("EEPROM 설정이 이미 목표값 — 토크를 건드리지 않는다");
            return Ok(());
        }
        // EEPROM 쓰기 전에는 토크를 반드시 내려야 한다. 프로세스 로컬 `torque_enabled`는 새
        // 실행에서 항상 false라 물리 상태를 못 믿는다 — 조건 없이 보낸다.
        self.enable_torque(false)?;
        self.write_operating_mode()?;
        self.write_max_pwm_limits()?;
        self.write_max_current_limits()?;
        return Ok(());
    }

    /// Operating Mode · PWM Limit · Current Limit이 모두 목표값인가.
    ///
    /// 읽기에 실패하면 `false` — 모르면 쓰는 쪽이 안전하다.
    fn position_mode_already_configured(&mut self) -> Result<bool, HwError> {
        // `real` 없이 빌드하면 아래 Real 갈래가 통째로 사라져 이 값이 안 쓰인다.
        #[cfg(feature = "real")]
        let config = self.mapping.config.clone();
        match &mut self.backend {
            // dry-run은 실제 레지스터가 없다 — 항상 쓰기 경로를 타 기존 검증을 유지한다.
            BusBackend::DryRun { .. } => return Ok(false),
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids = config.bus_ids();
                let modes = read_u8s(real, &ids, config.addr_operating_mode, &config);
                if !modes.is_some_and(|v| v.iter().all(|m| *m == config.operating_mode)) {
                    return Ok(false);
                }
                let pwm = read_u16s(real, &ids, config.addr_pwm_limit, &config);
                if !pwm.is_some_and(|v| v.iter().all(|p| *p == config.pwm_limit_max)) {
                    return Ok(false);
                }
                if !config.current_limit_max_by_id.is_empty() {
                    let current_ids: Vec<u8> = config
                        .current_limit_max_by_id
                        .iter()
                        .map(|(id, _)| *id)
                        .collect();
                    let want: Vec<u16> = config
                        .current_limit_max_by_id
                        .iter()
                        .map(|(_, v)| *v)
                        .collect();
                    let got = read_u16s(real, &current_ids, config.addr_current_limit, &config);
                    if !got.is_some_and(|v| v == want) {
                        return Ok(false);
                    }
                }
                return Ok(true);
            }
        }
    }

    fn write_operating_mode(&mut self) -> Result<(), HwError> {
        let mode = self.mapping.config.operating_mode;
        match &mut self.backend {
            BusBackend::DryRun {
                last_operating_mode,
                ..
            } => {
                *last_operating_mode = Some(mode);
            }
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
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

    fn write_max_pwm_limits(&mut self) -> Result<(), HwError> {
        let limit = self.mapping.config.pwm_limit_max;
        let ids = self.mapping.config.bus_ids();
        let limits: Vec<(u8, u16)> = ids.iter().map(|&id| (id, limit)).collect();
        match &mut self.backend {
            BusBackend::DryRun {
                last_pwm_limits, ..
            } => {
                *last_pwm_limits = limits;
            }
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let address = self.mapping.config.addr_pwm_limit;
                let data: Vec<Vec<u8>> = limits.iter().map(|(_, v)| pack_u16(*v)).collect();
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|error| {
                        read_transport_error(format!(
                            "PWM Limit sync_write 실패 (addr={address}): {error}"
                        ))
                    })?;
            }
        }
        return Ok(());
    }

    fn write_max_current_limits(&mut self) -> Result<(), HwError> {
        let limits = self.mapping.config.current_limit_max_by_id.clone();
        if limits.is_empty() {
            return Ok(());
        }
        match &mut self.backend {
            BusBackend::DryRun {
                last_current_limits,
                ..
            } => {
                *last_current_limits = limits;
            }
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let address = self.mapping.config.addr_current_limit;
                let ids: Vec<u8> = limits.iter().map(|(id, _)| *id).collect();
                let data: Vec<Vec<u8>> = limits.iter().map(|(_, v)| pack_u16(*v)).collect();
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                real.sync_write_with_retry(&ids, address, &data, retries, retry_delay_ms)
                    .map_err(|error| {
                        read_transport_error(format!(
                            "Current Limit sync_write 실패 (addr={address}): {error}"
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
        // 기본은 **토크를 켠 채로 둔다** — 끄면 프로그램이 끝나는 순간 팔이 중력으로 주저앉아
        // 링크·라켓이 상한다. AXL 레일도 같은 이유로 서보를 켠 채 닫는다(`AxlLive::drop`).
        // 손으로 팔을 움직이려면 `hold_torque_on_close = false`로 열거나 전원을 내린다.
        if self.torque_enabled && !self.mapping.config.hold_torque_on_close {
            let _ = self.enable_torque(false);
        }
    }
}

/// EEPROM 1바이트 레지스터를 모든 id에서 읽는다. 실패하면 `None`.
#[cfg(feature = "real")]
fn read_u8s(
    real: &mut RealBackend,
    ids: &[u8],
    address: u8,
    config: &DynamixelConfig,
) -> Option<Vec<u8>> {
    let raw = real
        .sync_read_with_retry(
            ids,
            address,
            1,
            config.comm_retries,
            config.comm_retry_delay_ms,
        )
        .ok()?;
    return raw
        .into_iter()
        .map(|bytes| bytes.first().copied())
        .collect();
}

/// EEPROM 2바이트 레지스터를 모든 id에서 읽는다. 실패하면 `None`.
#[cfg(feature = "real")]
fn read_u16s(
    real: &mut RealBackend,
    ids: &[u8],
    address: u8,
    config: &DynamixelConfig,
) -> Option<Vec<u16>> {
    let raw = real
        .sync_read_with_retry(
            ids,
            address,
            2,
            config.comm_retries,
            config.comm_retry_delay_ms,
        )
        .ok()?;
    return raw
        .into_iter()
        .map(|bytes| {
            let pair: [u8; 2] = bytes.as_slice().try_into().ok()?;
            Some(u16::from_le_bytes(pair))
        })
        .collect();
}
