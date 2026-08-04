//! 접수 계획 — 인터셉트·bang-bang·Magnus 휴리스틱.

use crate::robot::motion::InterceptWindow;

/// 인터셉트 샘플 상한.
pub const MAX_INTERCEPT_SAMPLES: usize = 1_024;

/// Magnus |ω| 클립 [rad/s].
pub const MAGNUS_OMEGA_MAX: f64 = 80.0;

/// 실기 AXL 레일 가속/감속 [m/s²] — `RailConfig::default()`도 이 값을 쓴다.
pub const RAIL_ACCEL_M_S2: f64 = 16.0;
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

/// 발사기 반복 시험용 고정 푸시가 한계를 지키며 임팩트를 만들 최소 시간.
pub const FIXED_IMPACT_MIN_DURATION_SECS: f64 = 0.25;
/// 라켓 면 법선 방향의 짧은 임팩트 전진 거리 [m].
pub const FIXED_IMPACT_PUSH_DISTANCE_M: f64 = 0.050;
/// 임팩트 순간 목표 라켓 선속도 [m/s].
pub const FIXED_IMPACT_PUSH_SPEED_M_S: f64 = 1.80;

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

/// coarse 선추종에서 임팩트 자세 쪽으로 미리 옮길 비율.
///
/// 값의 근거(관절별 차등 vs 균일, 통과 평면 수 계측표)는 이 값을 처음 정한
/// `sim::physics::world::COARSE_TRACK_JOINT_FRACTION`의 문서와
/// `docs/wp10-coarse-track-per-joint.md`에 있다. sim과 real이 **같은 값**을 써야
/// 해서 여기로 뺐다 — 다르면 sim에서 맞춘 커밋 타이밍이 실기에서 어긋난다.
pub const COARSE_TRACK_JOINT_FRACTION: f64 = 0.80;
