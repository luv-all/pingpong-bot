//! 스윙·관절 제어 한계.

/// 스윙을 시작하기 위해 필요한 최소 리드 타임 [s].
pub const MIN_SWING_SECS: f64 = 0.08;

/// 권장 스윙 궤적 길이 [s] — commit 창 중앙 근처.
pub const SWING_DURATION_SECS: f64 = 0.15;

/// 스윙 commit 상한 [s].
///
/// 이보다 긴 lead면 대기한다 (발사 직후 전 비행 구간 quintic → 조기 완료 방지).
/// 실제 duration은 commit 시점의 `time_to_impact`를 쓴다.
pub const SWING_COMMIT_MAX_SECS: f64 = 0.20;

/// §7.4 실행 가능성 근사 — 관절 각가속도 상한 [rad/s²] (토크 모델 전).
pub const MAX_JOINT_ACCEL: f64 = 120.0;

/// §7.4 대각 관성 근사 토크 상한 [N·m] (관절당, 시뮬).
pub const MAX_JOINT_TORQUE: f64 = 12.0;

/// 관절 유효 관성 근사 [kg·m²] (토크 ≈ I α, 링크별 동일 스텁).
pub const JOINT_INERTIA: f64 = 0.05;

/// 라켓 면 기본 open pitch [rad] — 손목 관절 초기각.
/// decisions D1: 이제 관절로 조절, 이 값은 default만.
pub const RACKET_OPEN_PITCH: f64 = 0.45;
