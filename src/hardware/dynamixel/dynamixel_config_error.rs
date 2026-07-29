use thiserror::Error;

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
