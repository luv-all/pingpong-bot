//! 탁구공, 접촉 계수.

/// 공 반지름 [m].
pub const RADIUS: f64 = 0.02;

/// 라켓-공, 테이블-공 반발계수. 측정 전 기본값.
/// tools/measure_restitution 으로 갱신.
pub const RESTITUTION: f64 = 0.85;

/// 테이블 바운스 반발. RESTITUTION 과 같다.
pub const TABLE_BOUNCE_RESTITUTION: f64 = RESTITUTION;

/// 테이블 바운스 접선 마찰 mu. 바운스 후 vt' = (1-mu) * vt.
/// tools/measure_friction 으로 갱신. 측정 전 잠정값.
pub const TABLE_BOUNCE_FRICTION: f64 = 0.15;
