//! 라켓/링크 충돌·뷰어 근사 치수.
//!
//! 기준: ITTF 규격 + `assets/robots/4-dof` CAD bbox (STL mm × 0.001).
//! primitive stick-figure와 OBB/Rapier가 같은 상수를 쓴다.

/// 라켓 블레이드 OBB half-extents [m] (충돌·Rapier cuboid).
///
/// CAD `pingpong_paddle_v5_1` bbox는 150×23×250 mm로 **손잡이까지** 포함된다.
/// 충돌·타격은 면만 쓴다 (손잡이→블레이드는 EE 오프셋).
/// ITTF 블레이드 면 ≈ 15 cm × 16 cm, 블레이드+러버 ≈ 1 cm.
pub const RACKET_HALF_X: f64 = 0.075;
pub const RACKET_HALF_Y: f64 = 0.08;
/// 면 법선(local +Z) 방향 반두께.
pub const RACKET_HALF_Z: f64 = 0.005;

/// 블레이드 원판 반경 [m]. primitive 뷰어 디스크용 (ITTF 직경 ~15 cm).
pub const RACKET_BLADE_RADIUS: f64 = 0.075;

/// 상완 링크 단면 반경 [m]. CAD `arm_v9_1` ≈ 47×28×97 mm, MX-64 ≈ 40×61×41.
pub const LINK_UPPER_RADIUS: f64 = 0.020;

/// 전완 링크 단면 반경 [m]. CAD `arm2_v2_1` ≈ 30×80×30 mm.
pub const LINK_FOREARM_RADIUS: f64 = 0.015;

/// 관절 마커 구 반경 [m]. MX-28/64 본체 스케일.
pub const JOINT_MARKER_RADIUS: f64 = 0.020;

/// 베이스 실린더 반경 [m]. CAD `base_link` ≈ 155×56×73 mm (이중 MX-64 폭의 절반 근사).
pub const ARM_BASE_RADIUS: f64 = 0.05;
/// 베이스 실린더 높이 [m].
pub const ARM_BASE_HEIGHT: f64 = 0.07;

/// 레일 시각화 단면 (너비×높이) [m]. 기구학 레일과 별개 장식.
pub const RAIL_VISUAL_WIDTH: f64 = 0.06;
pub const RAIL_VISUAL_HEIGHT: f64 = 0.04;

/// 테이블 면 위 최소 여유 [m] - OBB 최저점이 이보다 낮으면 관통.
pub const TABLE_CLEARANCE: f64 = 0.003;

/// `clamp_above_table` 최대 반복 (리프트->재IK).
pub const TABLE_CLAMP_ITERS: usize = 6;
