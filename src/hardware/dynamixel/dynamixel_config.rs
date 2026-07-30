use super::dynamixel_config_error::DynamixelConfigError;
use super::mirror_slave::MirrorSlave;

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
    /// Operating Mode 레지스터 (MX = 11).
    pub addr_operating_mode: u8,
    /// Position Control Mode = 3.
    pub operating_mode: u8,
    /// PWM Limit 레지스터 (MX = 36). Position 모드에서 실질 출력 상한.
    pub addr_pwm_limit: u8,
    /// PWM Limit 최대값 (100% = 885, MX-28/64 공통).
    pub pwm_limit_max: u16,
    /// Current Limit 레지스터 (MX-64 = 38). MX-28에는 없음.
    pub addr_current_limit: u8,
    /// MX-64 버스 ID별 Current Limit 최대. MX-28 ID는 넣지 않는다.
    pub current_limit_max_by_id: Vec<(u8, u16)>,
    pub profile_acceleration: u32,
    pub profile_velocity: u32,
    pub comm_retries: u32,
    pub comm_retry_delay_ms: u64,
    pub stream_hz: f64,
    pub joint_signs: Vec<i8>,
    pub joint_offsets_rad: Vec<f64>,
    pub motor_angle_limits_deg: Vec<[f64; 2]>,
    pub mirror_slaves: Vec<MirrorSlave>,
    /// 버스를 닫을 때 토크를 **켠 채로 둘지**.
    ///
    /// 기본 `true` — 끄면 프로그램이 끝나는 순간 팔이 중력으로 주저앉는다. AXL 레일도 같은
    /// 이유로 서보를 켠 채 닫는다(`AxlLive::drop`). `false`면 종료 시 힘을 빼 손으로 옮길 수
    /// 있지만, 그 전에 팔을 안전한 자세로 내려두어야 한다.
    pub hold_torque_on_close: bool,
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
