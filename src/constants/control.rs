//! 스윙, 관절 제어 한계.

/// 스윙을 시작하기 위해 필요한 최소 리드 타임 [s].
pub const MIN_SWING_SECS: f64 = 0.08;

/// 스윙 commit 상한 [s].
///
/// lead 가 이보다 길면 기다린다. 발사 직후 전 비행을 quintic 으로 잡으면
/// 너무 일찍 끝나기 쉽다. 실제 duration 은 commit 시점 time_to_impact.
pub const SWING_COMMIT_MAX_SECS: f64 = 0.35;

/// 임팩트 뒤 라켓 속도를 연속적으로 감속하는 팔로스루 시간 [s].
pub const SWING_FOLLOW_THROUGH_SECS: f64 = 0.06;

/// 스윙 commit 허용 최대 공 y. LENGTH_Y 대비 비율 (네트 지난 뒤).
///
/// ground truth / EKF control 공통. 상대 코트에서는 탄도 추정이 아직 흔들린다.
pub const SWING_COMMIT_MAX_BALL_Y_FRAC: f64 = 0.55;

/// 측정이 예측에서 이 거리[m] 이상 벗어나면 EKF 하드 리셋.
/// 주차에서 발사로 텔레포트할 때 등에 쓴다.
pub const EKF_MEAS_JUMP_M: f64 = 0.6;

/// 관절 각가속도 상한 [rad/s^2]. 토크 모델 붙이기 전 실행 가능성 근사.
pub const MAX_JOINT_ACCEL: f64 = 400.0;

/// 대각 관성 근사 토크 상한 [N*m]. 관절당, 시뮬용.
pub const MAX_JOINT_TORQUE: f64 = 20.0;

/// 관절 유효 관성 근사 [kg*m^2]. torque ~= I * alpha, 링크마다 같은 스텁.
pub const JOINT_INERTIA: f64 = 0.05;

/// 라켓 면 기본 open pitch [rad]. 손목 관절 초기값.
pub const RACKET_OPEN_PITCH: f64 = 0.45;
