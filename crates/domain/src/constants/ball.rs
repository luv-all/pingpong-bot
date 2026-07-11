//! 탁구공·접촉 계수.

/// 공 반지름 [m].
pub const RADIUS: f64 = 0.02;

/// 라켓-공·테이블-공 반발계수 (측정 전 기본값 — decisions E3 단일화).
pub const RESTITUTION: f64 = 0.85;

/// 테이블 바운스 반발 — [`RESTITUTION`]과 동일 (E3).
pub const TABLE_BOUNCE_RESTITUTION: f64 = RESTITUTION;
