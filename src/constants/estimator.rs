//! 궤적 추정/hit-plane 적분.

/// hit-plane 예측 최소 리드 [s].
pub const MIN_LEAD: f64 = 0.05;

/// hit-plane 예측 최대 리드 [s].
pub const MAX_LEAD: f64 = 1.2;

/// 탄도 적분 스텝 [s] - Rapier 1 kHz와 맞춤.
pub const INTEGRATE_DT: f64 = 0.001;

/// 접수 가능 최소 접근 속도 |v_y| [m/s] - 테이블 위 구름/잔여 드리프트 제외.
pub const MIN_APPROACH_SPEED_Y: f64 = 0.8;

/// 테이블 안착 높이 위 여유 [m] - 이보다 낮으면 굴림/바닥 스침으로 보고 예측 안 함.
pub const MIN_STRIKE_CLEARANCE: f64 = 0.05;

/// EKF 과정 잡음 - 위치 [m^2] 스케일 (x dt).
pub const Q_POS: f64 = 1e-4;

/// EKF 과정 잡음 - 속도 [(m/s)^2] 스케일 (x dt).
pub const Q_VEL: f64 = 1e-2;

/// EKF 측정 잡음 분산 [m^2] (삼각측량 ~3 cm).
pub const R_MEAS: f64 = 0.03 * 0.03;
