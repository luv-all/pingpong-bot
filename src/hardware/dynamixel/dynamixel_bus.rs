use tracing::{debug, info, warn};

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
    /// 전체 Present Position SyncRead가 깨진 버스에서 이후 느린 재시도를 반복하지 않는다.
    prefer_individual_position_reads: bool,
    /// 클램프 경고 스로틀 — 200 Hz 스트리밍이라 매번 찍으면 로그가 잠긴다.
    last_clamp_warn: Option<std::time::Instant>,
    /// 시작 실측이 소프트 한계 밖일 때만 여는 단방향 복귀 통로.
    /// 매 명령마다 바깥 경계를 안쪽으로 좁혀 다시 바깥으로 움직일 수 없게 한다.
    limit_escapes: Vec<Option<LimitEscape>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LimitEscape {
    Below { floor_tick: i32 },
    Above { ceiling_tick: i32 },
}

/// 모터 한계 클램프 경고 주기.
const CLAMP_WARN_PERIOD: std::time::Duration = std::time::Duration::from_secs(1);
/// 듀얼 모터가 약 3.5° 이상 어긋나면 기계적으로 싸우거나 영점이 틀린 것으로 본다.
const MIRROR_ALIGNMENT_MAX_ERROR_TICKS: i32 = 40;

/// 전체 SyncRead가 불안정한 버스에서 모터별 일반 Read로
/// Present Position을 읽는다. ID 하나에 broadcast SyncRead를 쓰지 않는다.
#[cfg(feature = "real")]
fn read_present_positions_individually(
    real: &mut RealBackend,
    ids: &[u8],
    address: u8,
    retries: u32,
    retry_delay_ms: u64,
    group_error: Option<&str>,
) -> Result<Vec<Vec<u8>>, HwError> {
    let mut recovered = Vec::with_capacity(ids.len());
    for id in ids {
        let bytes = real
            .read_with_retry(*id, address, 4, retries, retry_delay_ms)
            .map_err(|error| {
                let group_context = group_error.map_or_else(String::new, |group| {
                    format!(", group_error={group}")
                });
                read_transport_error(format!(
                    "Present Position ID별 read 실패 (addr={address}, id={id}{group_context}): {error}"
                ))
            })?;
        recovered.push(bytes);
    }
    return Ok(recovered);
}

impl DynamixelBus {
    pub fn dry_run(config: DynamixelConfig) -> Result<Self, DynamixelConfigError> {
        let mapping = MotorMapping::new(config)?;
        let joint_count = mapping.config.motor_ids.len();
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
            prefer_individual_position_reads: false,
            last_clamp_warn: None,
            limit_escapes: vec![None; joint_count],
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

    /// 미러 슬레이브의 실측 tick이
    /// `2*zero-master+조립 영점 보정`과 맞는지 검사한다.
    /// 이 검사는 시작·중립 복귀 직후에만 호출해 실시간 명령 경로를 느리게 하지 않는다.
    pub fn verify_mirror_alignment(&mut self) -> Result<(), HwError> {
        let config = self.mapping.config.clone();
        if config.mirror_slaves.is_empty() {
            return Ok(());
        }
        match &mut self.backend {
            BusBackend::DryRun { .. } => return Ok(()),
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                for pair in &config.mirror_slaves {
                    let ids = [pair.master_id, pair.slave_id];
                    let raw = read_present_positions_individually(
                        real,
                        &ids,
                        config.addr_present_position,
                        config.comm_retries,
                        config.comm_retry_delay_ms,
                        None,
                    )?;
                    let mut ticks = Vec::with_capacity(2);
                    for bytes in raw {
                        let bytes: [u8; 4] = bytes.as_slice().try_into().map_err(|_| {
                            read_transport_error("듀얼 모터 Present Position 응답 길이 오류")
                        })?;
                        ticks.push(u32::from_le_bytes(bytes) as i32);
                    }
                    let master_tick = ticks[0];
                    let slave_tick = ticks[1];
                    let expected_slave_tick = config.mirror_tick(master_tick);
                    let slave_minus_expected_tick = slave_tick - expected_slave_tick;
                    info!(
                        master_id = pair.master_id,
                        slave_id = pair.slave_id,
                        master_tick,
                        slave_tick,
                        expected_slave_tick,
                        slave_minus_expected_tick,
                        "듀얼 MX-64 실측 대칭 진단"
                    );
                    if slave_minus_expected_tick.abs() > MIRROR_ALIGNMENT_MAX_ERROR_TICKS {
                        return Err(read_transport_error(format!(
                            "듀얼 MX-64 정렬 불일치: ID{}={}tick, ID{}={}tick, 기대={}tick, 오차={:+}tick. 방향·혼 영점·체결을 확인할 때까지 구동 차단",
                            pair.master_id,
                            master_tick,
                            pair.slave_id,
                            slave_tick,
                            expected_slave_tick,
                            slave_minus_expected_tick,
                        )));
                    }
                }
            }
        }
        return Ok(());
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
        let joint_count = mapping.config.motor_ids.len();
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
        // Windows에서 COM 포트를 열면 USB 어댑터의 DTR/RTS 상태가
        // 바뀌며 수신 버퍼에 불완전한 바이트가 남을 수 있다. 모터에
        // 아무 명령도 보내지 않고 잠시 기다린 뒤 RX만 비워 첫 안전
        // 정렬 검사가 잔여 패킷을 읽지 않게 한다.
        std::thread::sleep(std::time::Duration::from_millis(200));
        port.clear(serialport::ClearBuffer::Input)
            .map_err(|error| {
                read_transport_error(format!("시리얼 포트 초기 RX 버퍼 정리 실패: {error}"))
            })?;
        let bus = Self {
            mapping,
            backend: BusBackend::Real(RealBackend::new(
                rustypot::DynamixelProtocolHandler::v2(),
                port,
            )),
            torque_enabled: false,
            prefer_individual_position_reads: false,
            last_clamp_warn: None,
            limit_escapes: vec![None; joint_count],
        };
        return Ok(bus);
    }

    /// Python `enable_torque`: Torque ON이면 profile 재적용 후 Torque Enable SyncWrite.
    ///
    /// Rust 추가 안전: Torque ON 직전 **버스의 각 모터** Present를 같은
    /// 모터의 Goal에 복사해 잔여 Goal 급기동을 막는다. 미러 슬레이브도
    /// 마스터에서 계산하지 않고 자신의 Present를 쓴다. 정렬 오류가 있는
    /// 상태에서 계산된 미러 목표를 먼저 쓰면 Torque ON과 동시에 급회전한다.
    pub fn enable_torque(&mut self, enabled: bool) -> Result<(), HwError> {
        if enabled {
            self.apply_motion_profile()?;
            self.hold_each_motor_at_present_position()?;
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

    /// Torque ON 직전 각 버스 ID의 Goal을 해당 ID의 Present로 맞춘다.
    ///
    /// 일반 관절 명령은 ID1에서 ID2 미러 목표를 만들어야 하지만, 토크를
    /// 처음 걸 때는 구동이 아닌 현재 자세 유지가 목적이다. 따라서 ID2의
    /// 실측값을 버리고 `mirror_tick(ID1)`을 쓰면 안 된다.
    fn hold_each_motor_at_present_position(&mut self) -> Result<(), HwError> {
        let dry_run_goals = match &self.backend {
            BusBackend::DryRun { ticks, .. } => Some(self.expand_goal_ticks(ticks)),
            #[cfg(feature = "real")]
            BusBackend::Real(_) => None,
        };
        match &mut self.backend {
            BusBackend::DryRun { last_bus_goals, .. } => {
                // dry-run은 슬레이브 Present를 별도로 보관하지 않으므로 정렬된
                // 실기와 같이 논리 관절값에서 미러 ID만 확장한다.
                *last_bus_goals = dry_run_goals.expect("dry-run goals must be prepared");
            }
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let config = self.mapping.config.clone();
                let ids = config.bus_ids();
                let raw_positions = read_present_positions_individually(
                    real,
                    &ids,
                    config.addr_present_position,
                    config.comm_retries,
                    config.comm_retry_delay_ms,
                    None,
                )?;
                let data = raw_positions
                    .into_iter()
                    .map(|bytes| {
                        let raw: [u8; 4] = bytes.as_slice().try_into().map_err(|_| {
                            read_transport_error(format!(
                                "Torque ON 전 Present Position 응답 길이 오류: got {} bytes, want 4",
                                bytes.len()
                            ))
                        })?;
                        Ok(pack_u32(u32::from_le_bytes(raw)))
                    })
                    .collect::<Result<Vec<_>, HwError>>()?;
                real.sync_write_with_retry(
                    &ids,
                    config.addr_goal_position,
                    &data,
                    config.comm_retries,
                    config.comm_retry_delay_ms,
                )
                .map_err(|error| {
                    command_transport_error(
                        0.0,
                        ids.len(),
                        format!(
                            "Torque ON 전 ID별 Goal=Present sync_write 실패 (addr={}): {error}",
                            config.addr_goal_position
                        ),
                    )
                })?;
            }
        }
        Ok(())
    }

    /// Goal/Present Position과 Torque Enable, Hardware Error Status를 ID별로 기록한다.
    ///
    /// SyncWrite 성공은 패킷을 보냈다는 뜻일 뿐 모터가 토크를 유지한다는 뜻은 아니다.
    /// 특히 토크가 켜진 팔을 손으로 밀면 overload shutdown으로 Goal은 갱신돼도
    /// Present가 따라오지 않을 수 있어 시작 자세 실패 시 이 네 값을 같이 봐야 한다.
    pub fn log_joint_diagnostics(&mut self) {
        match &mut self.backend {
            BusBackend::DryRun { ticks, .. } => {
                debug!(present_ticks = ?ticks, "Dynamixel dry-run 관절 진단");
            }
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                const HARDWARE_ERROR_STATUS_ADDRESS: u8 = 70;
                let config = self.mapping.config.clone();
                for id in config.bus_ids() {
                    // 미러 slave 하나가 응답하지 않아도 나머지 관절의 원인을 볼 수
                    // 있도록 SyncRead를 ID별로 수행한다.
                    let torque = read_u8(real, id, config.addr_torque_enable, &config);
                    let error = read_u8(real, id, HARDWARE_ERROR_STATUS_ADDRESS, &config);
                    let goal_tick = read_u32(real, id, config.addr_goal_position, &config);
                    let present_tick = read_u32(real, id, config.addr_present_position, &config);
                    let tick_error = goal_tick
                        .zip(present_tick)
                        .map(|(goal, present)| i64::from(goal) - i64::from(present));
                    if torque.is_none()
                        || error.is_none()
                        || goal_tick.is_none()
                        || present_tick.is_none()
                    {
                        warn!(
                            id,
                            torque_enabled = ?torque,
                            hardware_error_status = ?error,
                            goal_tick = ?goal_tick,
                            present_tick = ?present_tick,
                            "Dynamixel 관절 진단 일부 읽기 실패 — 해당 ID 배선·전원·통신 확인"
                        );
                    } else if torque == Some(0)
                        || error.is_some_and(|status| status != 0)
                        || tick_error.is_some_and(|ticks| ticks.abs() > 20)
                    {
                        let error = error.unwrap_or_default();
                        warn!(
                            id,
                            torque_enabled = ?torque,
                            hardware_error_status = error,
                            hardware_error = hardware_error_labels(error),
                            goal_tick = ?goal_tick,
                            present_tick = ?present_tick,
                            goal_minus_present_tick = ?tick_error,
                            "Dynamixel 관절 이상 진단"
                        );
                    } else {
                        info!(
                            id,
                            torque_enabled = ?torque,
                            hardware_error_status = ?error,
                            goal_tick = ?goal_tick,
                            present_tick = ?present_tick,
                            goal_minus_present_tick = ?tick_error,
                            "Dynamixel 관절 정상 진단"
                        );
                    }
                }
            }
        }
    }

    /// 시작 자세가 수렴하지 않을 때 토크 차단·오류·큰 위치 괴리가 있는 모터를
    /// 재부팅하고 현재 위치에서 토크를 다시 건다. 호출측이 이후 목표를 재전송한다.
    pub fn recover_joint_control(&mut self) -> Result<bool, HwError> {
        match &mut self.backend {
            BusBackend::DryRun { .. } => return Ok(false),
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                const HARDWARE_ERROR_STATUS_ADDRESS: u8 = 70;
                let config = self.mapping.config.clone();
                let mut recover_ids = Vec::new();
                for id in config.bus_ids() {
                    let torque = read_u8(real, id, config.addr_torque_enable, &config);
                    let error = read_u8(real, id, HARDWARE_ERROR_STATUS_ADDRESS, &config);
                    if torque == Some(0) || error.is_some_and(|status| status != 0) {
                        // 둘 중 하나만 읽혔더라도 명시적인 shutdown 증거가 있으면
                        // 복구한다. 다른 레지스터의 읽기 실패 때문에 놓치지 않는다.
                        recover_ids.push(id);
                    } else if torque.is_none() && error.is_none() {
                        warn!(
                            id,
                            "Dynamixel 자동 복구 진단 불가 — 무응답 ID는 강제 구동하지 않음"
                        );
                    } else if torque.is_none() || error.is_none() {
                        warn!(
                            id,
                            torque_enabled = ?torque,
                            hardware_error_status = ?error,
                            "Dynamixel 자동 복구 진단 일부 실패 — 해당 ID는 강제 구동하지 않음"
                        );
                    }
                }
                if recover_ids.is_empty() {
                    return Ok(false);
                }
                warn!(ids = ?recover_ids, "Dynamixel 시작 자세 자동 복구 — 모터 재부팅");
                for id in &recover_ids {
                    real.reboot_with_retry(*id, config.comm_retries, config.comm_retry_delay_ms)
                        .map_err(|error| {
                            read_transport_error(format!("Dynamixel ID {id} reboot 실패: {error}"))
                        })?;
                    std::thread::sleep(std::time::Duration::from_millis(250));
                }
            }
        }
        // reboot 뒤 Torque Enable은 0이다. Present를 Goal로 먼저 복사한 뒤 토크를
        // 켜므로 갑자기 예전 Goal로 튀지 않는다.
        self.enable_torque(true)?;
        return Ok(true);
    }

    /// Python `set_joint_positions`.
    ///
    /// 모터 각도 한계에 걸리는 관절이 있으면 경고한다 — 플래너는 URDF 한계만 보므로
    /// `motor_angle_limits_deg`가 더 좁으면 여기서 조용히 잘려 팔이 명령과 다른 곳에 선다.
    /// 200 Hz 스트리밍이라 스로틀한다.
    pub fn write_joints(&mut self, joints: &Joints) -> Result<(), HwError> {
        let clamped: Vec<usize> = joints
            .values
            .iter()
            .enumerate()
            .filter(|(index, angle)| {
                self.mapping.clamped_by_motor_limit(*index, **angle)
                    && !self.limit_escape_allows(*index, **angle)
            })
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
        return self.write_joints_inner(joints);
    }

    /// 한 관절의 Goal Position만 보낸다.
    ///
    /// 레거시 직접 제어와 복구 로직에서 팔 전체를 다시 명령하지 않고
    /// 한 관절만 움직일 때 쓴다. 전체 SyncWrite를 재사용하면 나머지 관절도
    /// 다시 제어 대상이 되므로 별도 단일축 경로를 둔다.
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
            && !self.limit_escape_allows(joint_index, angle_rad)
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

        let tick = self.limit_aware_goal_tick(joint_index, angle_rad);
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

    fn write_joints_inner(&mut self, joints: &Joints) -> Result<(), HwError> {
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
            .map(|(index, angle)| self.limit_aware_goal_tick(index, *angle))
            .collect();
        self.write_raw_goal_ticks(&ticks, 0.0)
    }

    /// 현재 자세가 모터 소프트 한계 밖이면 그 현재값까지만 임시 허용한다.
    /// 이후 목표 tick은 정상 범위 방향으로만 갈 수 있고, 한계 안에 들어오는 순간
    /// 임시 통로를 닫는다. 따라서 정상 운전 범위 자체는 넓어지지 않는다.
    pub fn arm_limit_escape_from(&mut self, joints: &Joints) -> Result<(), HwError> {
        let joint_count = self.mapping.config.motor_ids.len();
        if joints.values.len() != joint_count {
            return Err(command_transport_error(
                0.0,
                joints.values.len(),
                format!(
                    "한계 복귀 관절 수 불일치: got {} want {joint_count}",
                    joints.values.len()
                ),
            ));
        }
        self.limit_escapes.fill(None);
        for (index, angle) in joints.values.iter().copied().enumerate() {
            let raw = self.mapping.radians_to_raw_ticks(index, angle);
            let (lo, hi) = self.mapping.tick_limit(index);
            let escape = if raw < lo {
                Some(LimitEscape::Below { floor_tick: raw })
            } else if raw > hi {
                Some(LimitEscape::Above { ceiling_tick: raw })
            } else {
                None
            };
            if let Some(escape) = escape {
                warn!(
                    joint = index,
                    motor_id = self.mapping.config.motor_ids[index],
                    present_tick = raw,
                    normal_min_tick = lo,
                    normal_max_tick = hi,
                    ?escape,
                    "시작 관절이 모터 한계 밖 — 현재값 유지 후 정상 범위 방향으로만 복귀"
                );
            }
            self.limit_escapes[index] = escape;
        }
        return Ok(());
    }

    fn limit_escape_allows(&self, joint_index: usize, angle_rad: f64) -> bool {
        let raw = self.mapping.radians_to_raw_ticks(joint_index, angle_rad);
        let (lo, hi) = self.mapping.tick_limit(joint_index);
        return match self.limit_escapes.get(joint_index).copied().flatten() {
            Some(LimitEscape::Below { floor_tick }) => raw >= floor_tick && raw < lo,
            Some(LimitEscape::Above { ceiling_tick }) => raw > hi && raw <= ceiling_tick,
            None => false,
        };
    }

    fn limit_aware_goal_tick(&mut self, joint_index: usize, angle_rad: f64) -> i32 {
        let raw = self.mapping.radians_to_raw_ticks(joint_index, angle_rad);
        let (lo, hi) = self.mapping.tick_limit(joint_index);
        let Some(escape) = self.limit_escapes[joint_index] else {
            return raw.clamp(lo, hi);
        };
        match escape {
            LimitEscape::Below { floor_tick } if raw < lo => {
                let applied = raw.max(floor_tick);
                self.limit_escapes[joint_index] = Some(LimitEscape::Below {
                    floor_tick: applied,
                });
                applied
            }
            LimitEscape::Above { ceiling_tick } if raw > hi => {
                let applied = raw.min(ceiling_tick);
                self.limit_escapes[joint_index] = Some(LimitEscape::Above {
                    ceiling_tick: applied,
                });
                applied
            }
            _ => {
                self.limit_escapes[joint_index] = None;
                info!(
                    joint = joint_index,
                    motor_id = self.mapping.config.motor_ids[joint_index],
                    "모터 한계 밖 시작 자세의 단방향 복귀 완료"
                );
                raw.clamp(lo, hi)
            }
        }
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
        let prefer_individual_position_reads = self.prefer_individual_position_reads;
        let ticks = match &mut self.backend {
            BusBackend::DryRun { ticks, .. } => ticks.clone(),
            #[cfg(feature = "real")]
            BusBackend::Real(real) => {
                let ids = self.mapping.config.motor_ids.clone();
                let address = self.mapping.config.addr_present_position;
                let retries = self.mapping.config.comm_retries;
                let retry_delay_ms = self.mapping.config.comm_retry_delay_ms;
                let raw_positions = if prefer_individual_position_reads {
                    read_present_positions_individually(
                        real,
                        &ids,
                        address,
                        retries,
                        retry_delay_ms,
                        None,
                    )?
                } else {
                    match real.sync_read_with_retry(&ids, address, 4, retries, retry_delay_ms) {
                        Ok(raw) => raw,
                        Err(group_error) => {
                            // 전체 SyncRead는 한 ID의 응답만 깨져도 전부 실패한다.
                            // 모터가 움직일 때의 일시적 Checksum/timeout은 ID별 읽기로
                            // 격리해 정상 응답 모터까지 함께 버리지 않는다.
                            warn!(
                                ids = ?ids,
                                error = %group_error,
                                "Present Position 전체 SyncRead 실패 — 이후 ID별 읽기 사용"
                            );
                            self.prefer_individual_position_reads = true;
                            let recovered = read_present_positions_individually(
                                real,
                                &ids,
                                address,
                                retries,
                                retry_delay_ms,
                                Some(&group_error),
                            )?;
                            info!(ids = ?ids, "Present Position ID별 읽기로 통신 복구");
                            recovered
                        }
                    }
                };
                raw_positions
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

/// 한 ID의 1바이트 레지스터를 읽는다. 다른 ID의 통신 장애에 영향받지 않는다.
#[cfg(feature = "real")]
fn read_u8(real: &mut RealBackend, id: u8, address: u8, config: &DynamixelConfig) -> Option<u8> {
    return read_u8s(real, &[id], address, config)?.into_iter().next();
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

#[cfg(feature = "real")]
fn read_u32s(
    real: &mut RealBackend,
    ids: &[u8],
    address: u8,
    config: &DynamixelConfig,
) -> Option<Vec<u32>> {
    let raw = real
        .sync_read_with_retry(
            ids,
            address,
            4,
            config.comm_retries,
            config.comm_retry_delay_ms,
        )
        .ok()?;
    return raw
        .into_iter()
        .map(|bytes| {
            let value: [u8; 4] = bytes.as_slice().try_into().ok()?;
            Some(u32::from_le_bytes(value))
        })
        .collect();
}

/// 한 ID의 4바이트 레지스터를 읽는다. 다른 ID의 통신 장애에 영향받지 않는다.
#[cfg(feature = "real")]
fn read_u32(real: &mut RealBackend, id: u8, address: u8, config: &DynamixelConfig) -> Option<u32> {
    return read_u32s(real, &[id], address, config)?.into_iter().next();
}

#[cfg(feature = "real")]
fn hardware_error_labels(status: u8) -> &'static str {
    if status & (1 << 5) != 0 {
        return "overload";
    }
    if status & (1 << 4) != 0 {
        return "electrical_shock";
    }
    if status & (1 << 3) != 0 {
        return "motor_encoder";
    }
    if status & (1 << 2) != 0 {
        return "overheating";
    }
    if status & 1 != 0 {
        return "input_voltage";
    }
    return "none";
}
