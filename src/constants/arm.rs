//! 경진용 `Arm::competition` 기하/관절 한계.
//!
//! DOF: yaw + 어깨 + 팔꿈치 + 손목 — `all-4-export.urdf` 직렬 체인과 동일.
//! competition primitive는 URDF origin 합산 길이를 쓴다 (≈0.07~0.15 m/세그먼트).

/// 베이스 y [m] - 테이블 끝에서 살짝 안쪽.
pub const BASE_Y: f64 = 0.02;

/// 관절 추종 최대 각속도 [rad/s] (시뮬).
pub const MAX_JOINT_SPEED: f64 = 16.0;

/// 리니어 레일 최대 속도 [m/s] (시뮬).
pub const RAIL_MAX_SPEED: f64 = 12.0;
