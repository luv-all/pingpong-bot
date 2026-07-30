//! 접수 계획 — 인터셉트·bang-bang·Magnus 휴리스틱.

use crate::motion::InterceptWindow;

/// 인터셉트 샘플 상한.
pub const MAX_INTERCEPT_SAMPLES: usize = 1_024;

/// Magnus |ω| 클립 [rad/s].
pub const MAGNUS_OMEGA_MAX: f64 = 80.0;

/// 실기 AXL 레일 가속/감속 [m/s²] — `RailConfig::default().accel`과 맞춤.
pub const RAIL_ACCEL_M_S2: f64 = 12.0;
pub const POSITION_TOLERANCE_RAD_OR_M: f64 = 1e-3;
pub const RACKET_SPEED_RATIO_TOLERANCE: f64 = 0.15;
pub const RACKET_DIRECTION_TOLERANCE_DEG: f64 = 15.0;
pub const PLAN_DT_SECS: f64 = 0.001;
pub const MAX_PLAN_TIME_SECS: f64 = 0.5;
pub const JACOBIAN_DAMPING: f64 = 0.05;
pub const TIME_TO_GO_BIAS: f64 = 0.5;
pub const MIN_TIME_TO_GO_SECS: f64 = 1e-3;
pub const JDOT_STEP: f64 = 1e-4;

pub const RETURN_TO_CENTER_MIN_SECS: f64 = 0.3;
pub const RETURN_TO_CENTER_MAX_SECS: f64 = 3.0;
pub const RETURN_TO_CENTER_GROWTH: f64 = 1.4;

impl Default for InterceptWindow {
    fn default() -> Self {
        // rail_frame behind≈0.10 기준 접수 창.
        return Self {
            y_min: 0.08,
            y_max: 0.35,
            sample_step: 0.03,
        };
    }
}
