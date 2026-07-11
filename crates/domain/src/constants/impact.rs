//! 임팩트·로프트 리턴 목표.

/// 네트 위 여유 높이 [m].
pub const NET_CLEARANCE: f64 = 0.08;

/// 임팩트 → 네트까지 목표 비행 시간 [s].
pub const LOFT_TIME_TO_NET: f64 = 0.40;

/// 리턴 속도 상한 [m/s].
pub const MAX_RETURN_SPEED: f64 = 6.0;

/// 레거시 협력 랠리: 입사 대비 출사 스케일.
pub const COOPERATIVE_RETURN_SCALE: f64 = 0.35;
