//! 경진용 `Arm::competition` 기하/관절 한계.
//!
//! DOF: yaw + 어깨 + 팔꿈치 + 손목 — `all-4-export.urdf` 직렬 체인과 동일.
//! `LINK_UPPER`/`LINK_FOREARM`/`LINK_WRIST_STUB`은 legacy planar Arm 빌더용.
//! competition primitive는 URDF origin 합산 길이를 쓴다 (≈0.07~0.15 m/세그먼트).

/// FK가 지원하는 revolute 축 수.
pub const SUPPORTED_FK_JOINTS: usize = 4;

/// 위치 IK에 쓰는 팔 링크 수 (마지막은 손목 스텁).
pub const ARM_POSITION_LINKS: usize = 2;

/// 베이스 y [m] - 테이블 끝에서 살짝 안쪽.
pub const BASE_Y: f64 = 0.02;

/// legacy planar 상완 길이 [m]. serial competition에서는 미사용.
pub const LINK_UPPER: f64 = 0.18;

/// legacy planar 전완 길이 [m]. serial competition에서는 미사용.
pub const LINK_FOREARM: f64 = 0.18;

/// legacy planar 손목 스텁 길이 [m].
pub const LINK_WRIST_STUB: f64 = 0.02;

/// yaw 관절 하한 [rad].
pub const YAW_MIN: f64 = -1.2;
/// yaw 관절 상한 [rad].
pub const YAW_MAX: f64 = 1.2;
/// yaw 초기각 [rad].
pub const YAW_DEFAULT: f64 = 0.0;

/// 어깨 관절 하한 [rad].
pub const SHOULDER_MIN: f64 = -0.2;
/// 어깨 관절 상한 [rad].
pub const SHOULDER_MAX: f64 = 1.4;
/// 어깨 초기각 [rad].
pub const SHOULDER_DEFAULT: f64 = 0.6;

/// 팔꿈치 관절 하한 [rad].
pub const ELBOW_MIN: f64 = -1.5;
/// 팔꿈치 관절 상한 [rad].
pub const ELBOW_MAX: f64 = 0.5;
/// 팔꿈치 초기각 [rad].
pub const ELBOW_DEFAULT: f64 = -0.4;

/// 손목 open 하한 [rad].
pub const WRIST_MIN: f64 = 0.05;
/// 손목 open 상한 [rad].
pub const WRIST_MAX: f64 = 1.2;

/// 관절 추종 최대 각속도 [rad/s] (시뮬).
pub const MAX_JOINT_SPEED: f64 = 16.0;

/// 리니어 레일 최대 속도 [m/s] (시뮬).
pub const RAIL_MAX_SPEED: f64 = 12.0;
