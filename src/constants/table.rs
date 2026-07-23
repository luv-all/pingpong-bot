//! ITTF 탁구대 규격 [m] - Z-up, 꼭짓점 원점.
//!
//! 원점 = 로봇 쪽 꼭짓점(바닥). +X = 너비(좁은 변), +Y = 길이(긴 변), +Z = 고도.

/// 플레이 면 너비 (x, 좁은 변).
pub const WIDTH_X: f64 = 1.525;
/// 플레이 면 길이 (y, 긴 변).
pub const LENGTH_Y: f64 = 2.74;
/// 테이블 윗면 z 좌표 (바닥 z=0 기준).
pub const SURFACE_Z: f64 = 0.76;
/// 테이블 두께의 절반.
pub const HALF_THICKNESS: f64 = 0.0125;
/// 네트 상단 높이 (테이블 면 기준). ITTF 15.25 cm.
pub const NET_HEIGHT: f64 = 0.1525;
/// 기본 접수 평면 y [m]. 로봇 앞에서 공을 맞출 깊이.
/// `defaults::rail_frame` 마운트(y=−0.20) + `defaults::arm` 도달(~0.38 m)에 맞춤.
pub const DEFAULT_HIT_PLANE_Y: f64 = 0.08;
