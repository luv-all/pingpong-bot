//! 임팩트/로프트 리턴 목표.

/// 네트 위 여유 높이 [m].
pub const NET_CLEARANCE: f64 = 0.08;

/// 임팩트에서 상대 코트 중앙 바운드까지 목표 비행 시간 [s].
pub const RALLY_TIME_TO_BOUNCE: f64 = 0.55;

/// Rapier의 유연 접촉을 포함해 라켓 명령 역산에 사용하는 유효 반발계수.
pub const RACKET_EFFECTIVE_RESTITUTION: f64 = 0.42;

/// 리턴 속도 상한 [m/s].
pub const MAX_RETURN_SPEED: f64 = 6.0;
