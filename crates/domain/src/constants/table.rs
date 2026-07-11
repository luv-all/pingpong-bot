//! ITTF 탁구대 규격 [m] — Z-up, 꼭짓점 원점.
//!
//! 원점 = 로봇 쪽 꼭짓점(바닥). **+X** = 너비(좁은 변), **+Y** = 길이(긴 변), **+Z** = 고도.

/// 플레이 면 너비 (x, 좁은 변).
pub const WIDTH_X: f64 = 1.525;
/// 플레이 면 길이 (y, 긴 변).
pub const LENGTH_Y: f64 = 2.74;
/// 테이블 윗면 z 좌표 (바닥 z=0 기준).
pub const SURFACE_Z: f64 = 0.76;
/// 테이블 두께의 절반.
pub const HALF_THICKNESS: f64 = 0.0125;
/// 네트 중심 높이 (테이블 면 기준).
pub const NET_HEIGHT: f64 = 0.08;
/// 기본 접수 평면 y [m] — 로봇(y≈0) 앞에서 공을 맞출 깊이.
/// `Arm::competition()` 최대 신장 ≈ 0.38 m 이내여야 한다 (구 0.65는 도달 불가).
pub const DEFAULT_HIT_PLANE_Y: f64 = 0.30;
