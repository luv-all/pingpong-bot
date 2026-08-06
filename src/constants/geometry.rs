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

/// 실측 라켓 장착 피치 보정 [rad].
///
/// 2026-08-05 시작 자세에서 엔코더 FK는 라켓 장축을 수직에서 18.55°로
/// 계산했지만, 실물은 손잡이 쪽이 로봇 방향으로 기울어진 8.00°였다.
/// CAD EE 자세에 local +X 기준 +10.55°를 더하면 두 방향이 일치한다.
pub const RACKET_MOUNT_PITCH_CORRECTION_RAD: f64 = 10.55_f64.to_radians();

/// URDF EE 원점 → 실물 블레이드 중심의 장축 방향 보정 거리 [m].
///
/// 같은 시작 자세에서 모델 블레이드 중심은 상판 위 0.3449 m였고, 실측
/// 최하단 0.155 m와 블레이드 길이 0.160 m, 기울기 8°로 구한 실제 중심은
/// 약 0.2349 m다. 따라서 보정된 local -Y(라켓 아래쪽)로 0.1111 m 옮긴다.
/// 이 값은 전체 라켓 길이 0.255 m를 충돌 박스로 키우는 값이 아니라, 실제
/// 타격면 중심을 올바르게 놓는 장착 보정이다.
pub const RACKET_BLADE_CENTER_OFFSET_M: f64 = 0.1111;

/// URDF 패들 링크 원점을 실측 블레이드 중심으로 바꾸는 고정 변환.
///
/// 반환값은 URDF 링크 좌표에서 합성하도록 `L * P * D * L⁻¹`이다.
/// `L`은 CAD 라켓 축을 domain 축으로 바꾸는 변환이고, `P`는
/// 실측 피치, `D`는 보정된 라켓 local -Y 방향 중심 이동이다.
pub fn racket_urdf_mount_calibration() -> nalgebra::Isometry3<f64> {
    use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

    let link_from_racket = UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
        std::f64::consts::PI,
    );
    let pitch = UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::x()),
        RACKET_MOUNT_PITCH_CORRECTION_RAD,
    );
    let link_from_racket_iso = Isometry3::from_parts(Translation3::identity(), link_from_racket);
    return link_from_racket_iso
        * Isometry3::from_parts(Translation3::identity(), pitch)
        * Isometry3::translation(0.0, -RACKET_BLADE_CENTER_OFFSET_M, 0.0)
        * link_from_racket_iso.inverse();
}

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

/// 테이블 면 위 최소 안전 여유 [m].
///
/// 실물의 조립 오차·링크 유격·Dynamixel 추종 오차를 흡수하기 위해 로봇 링크와
/// 라켓 OBB의 최저점이 상판보다 최소 3 cm 높게 유지되도록 한다. 예전 3 mm는
/// 시뮬 기하가 정확하다는 전제에 가까워 실기 충돌 방지 여유로는 부족했다.
pub const TABLE_CLEARANCE: f64 = 0.030;

/// `clamp_above_table` 최대 반복 (리프트->재IK).
pub const TABLE_CLAMP_ITERS: usize = 6;
