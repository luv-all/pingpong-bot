//! 라켓/링크 충돌·뷰어 근사 치수.
//!
//! 기준: ITTF 규격 + `assets/robots/4-dof` CAD bbox (STL mm × 0.001).
//! primitive stick-figure와 OBB/Rapier가 같은 상수를 쓴다.

/// 라켓 블레이드 OBB half-extents [m] (충돌·Rapier cuboid).
///
/// CAD `pingpong_paddle_v5_1` bbox는 150×23×250 mm로 **손잡이까지** 포함된다.
/// 충돌·타격은 면만 쓴다. 손잡이 길이는 [`RACKET_HANDLE_LENGTH`] (손목→면 중심).
/// ITTF 블레이드 면 ≈ 15 cm × 16 cm, 블레이드+러버 ≈ 1 cm.
pub const RACKET_HALF_X: f64 = 0.075;
pub const RACKET_HALF_Y: f64 = 0.08;
/// 면 법선(local +Z) 방향 반두께.
pub const RACKET_HALF_Z: f64 = 0.005;

/// 손목 조인트 → 블레이드 중심 (손잡이) [m].
///
/// 실기처럼 **면과 같은 평면**에서 원판 가장자리로 이어진다 (법선 방향 관통 아님).
/// CAD 라켓 링크: local +Y=면 법선, +Z=손잡이(면 내) → tip ≈ `(0, −HALF_Z, HANDLE)`.
/// CAD paddle 장축 ~0.25 m, 면 반경 0.075 → 조인트~면 중심 ≈ 0.10 m.
/// tip isometry: `(0, −HALF_Z, −HANDLE)` — local −Z가 홈 포즈에서 면내 손잡이.
pub const RACKET_HANDLE_LENGTH: f64 = 0.10;

/// `assets/robots/4-dof` URDF EE(`pingpong_paddle_v5_1`) local tip Y [m] (법선 축).
/// URDF 원점 보정용. primitive 손잡이 축은 local +Z — [`RACKET_HANDLE_LENGTH`].
pub const RACKET_URDF_TIP_Y: f64 = 0.0513;

/// 손잡이 시각·근사 반경 [m].
pub const RACKET_HANDLE_RADIUS: f64 = 0.012;

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

/// 레일 프로파일 두께(단면 높이) [m] — **실측(2026-07-30), 고정**.
///
/// 프로파일이 이미 제작돼 있어 조정 불가다. 베이스는 이 윗면에 얹히므로
/// [`crate::robot::RailFrame::mount_z`]가 하단 높이에 이 값을 더한다.
/// 이전 `RAIL_VISUAL_HEIGHT = 0.04`는 근거 없는 장식값이었다 — 실측이 있으니
/// 시각화도 이 상수를 쓴다.
pub const RAIL_THICKNESS: f64 = 0.055;

/// 레일 시각화 단면 너비 [m]. 단면 폭 실측이 없어 장식값으로 남긴다.
pub const RAIL_VISUAL_WIDTH: f64 = 0.06;

/// 로봇 아래 수평 알루미늄 프로파일의 충돌 모델 폭(Y) [m].
///
/// 사진에서 보이는 상부 레일·지지 프로파일 묶음을 단일 박스로
/// 보수적으로 감싼다. 뷰어 장식 폭(`RAIL_VISUAL_WIDTH`)과 달리
/// 실물 간섭 검사용이다.
pub const RAIL_FRAME_COLLISION_WIDTH: f64 = 0.16;

/// 철제 프레임에 두는 최소 여유 [m]. 케이블·프레임 모서리와
/// OBB 근사 오차를 포함해 링크가 실물 표면에 닿지 않게 한다.
pub const SUPPORT_FRAME_CLEARANCE: f64 = 0.02;

/// 테이블 면 위 최소 여유 [m] - OBB 최저점이 이보다 낮으면 관통.
pub const TABLE_CLEARANCE: f64 = 0.003;

/// `clamp_above_table` 최대 반복 (리프트->재IK).
pub const TABLE_CLAMP_ITERS: usize = 6;
