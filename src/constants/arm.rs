//! 경진용 `Arm::competition` 기하/관절 한계.
//!
//! DOF: yaw + 어깨 + 팔꿈치 + 손목 — `all-4-export.urdf` 직렬 체인과 동일.
//! competition primitive는 URDF origin 합산 길이를 쓴다 (≈0.07~0.15 m/세그먼트).

/// 베이스 y [m] - 테이블 끝에서 살짝 안쪽.
pub const BASE_Y: f64 = 0.02;

// 관절 추종 최대 각속도는 여기 없다 — 근거 없는 리터럴(`16.0`)이었던 것을 제거했다.
// 실기 Dynamixel 스펙 기반 SSOT는
// `crate::hardware::dynamixel::DYNAMIXEL_MAX_JOINT_SPEED_RAD_S`
// (`Arm::competition()`과 URDF 카탈로그 모두 공유; 근거: `.omc/research/dynamixel-specs.md`).

/// 리니어 레일 최대 속도 [m/s] (시뮬).
///
/// 이전 `12.0`은 근거 없는 리터럴이었다 — 테이블 전폭(1.525 m)을 0.127초에
/// 주파해, rough-to-fine 추종이 예측 방향으로 레일을 미리 옮길 때 렌더
/// 프레임 상 순간이동처럼 보였다(육안 확인, 2026-07-23). 실기
/// `config/real-hardware.toml`의 `[hardware.rail]` `vel`/`max_vel` = 5.0 m/s에
/// 맞춰 재보정 — 전폭 주파 0.305초로, 연속적인 움직임으로 보인다.
pub const RAIL_MAX_SPEED: f64 = 5.0;
