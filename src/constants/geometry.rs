//! 라켓/링크 충돌 근사(OBB) 치수 - sim Rapier/뷰어와 맞춤.

/// 4-dof paddle 단순 박스의 half-extents [m].
pub const RACKET_HALF_X: f64 = 0.075;
pub const RACKET_HALF_Y: f64 = 0.125;
/// 면 법선(local +Z) 방향 반두께.
pub const RACKET_HALF_Z: f64 = 0.0114;

/// 상완 링크 단면 반경 근사 [m] (뷰어 실린더 0.025).
pub const LINK_UPPER_RADIUS: f64 = 0.025;

/// 전완 링크 단면 반경 근사 [m] (뷰어 실린더 0.022).
pub const LINK_FOREARM_RADIUS: f64 = 0.022;

/// 테이블 면 위 최소 여유 [m] - OBB 최저점이 이보다 낮으면 관통.
pub const TABLE_CLEARANCE: f64 = 0.003;

/// `clamp_above_table` 최대 반복 (리프트->재IK).
pub const TABLE_CLAMP_ITERS: usize = 6;
