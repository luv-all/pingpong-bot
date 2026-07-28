//! Dynamixel 연속 구동 한계 (엔지니어링 가정).
//!
//! datasheet stall/RPM은 [`crate::constants::dynamixel`].

use crate::constants::dynamixel::{
    MX28_NO_LOAD_SPEED_RPM, MX28_STALL_TORQUE_NM, MX64_STALL_TORQUE_NM, rev_min_to_rad_s,
};

/// stall → 연속 토크 안전 한계 감쇠 (실측 확인 필요).
pub const CONTINUOUS_TORQUE_DERATE: f64 = 0.5;

/// 무부하 RPM 대비 지속 관절 속도 감쇠.
pub const JOINT_SPEED_DERATE: f64 = 0.5;

/// 실기 4축 관절 속도 상한 [rad/s] — MX-28 무부하 × [`JOINT_SPEED_DERATE`].
pub const DYNAMIXEL_MAX_JOINT_SPEED_RAD_S: f64 =
    rev_min_to_rad_s(MX28_NO_LOAD_SPEED_RPM) * JOINT_SPEED_DERATE;

/// 4-dof 관절별 연속 토크 안전 한계 [N·m].
///
/// joint0=yaw=MX-64×2(듀얼), joint1=shoulder=MX-64, joint2/3=MX-28.
pub fn joint_torque_limits_4dof_array() -> [f64; 4] {
    return [
        2.0 * MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
        MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
        MX28_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
        MX28_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
    ];
}

pub fn joint_torque_limits_4dof() -> Vec<f64> {
    return joint_torque_limits_4dof_array().to_vec();
}
