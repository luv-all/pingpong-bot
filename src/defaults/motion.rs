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
/// 공이 없을 때 기존 임팩트 자세보다 뒤에서 대기할 거리 [m].
///
/// 큰 동작을 줄인 반복 발사기 시험값 2cm.
pub const READY_PREWIND_DISTANCE_M: f64 = 0.020;
/// 공이 없을 때 미리 맞춰 둘 대표 라켓 중심 높이 [m].
pub const READY_RACKET_HEIGHT_M: f64 = 1.050;
/// 기본 인터셉트 구간 중앙의 준비 타격 y [m].
pub const READY_RACKET_Y_M: f64 = 0.215;
/// 공 검출 후 공 높이에서 유지할 임팩트 자세 기준 백스윙 거리 [m].
/// 준비 자세와 같은 2cm로 두어 검출 뒤 불필요한 추가 감김을 없앤다.
pub const DETECTION_WINDUP_DISTANCE_M: f64 = 0.020;
/// 기본 타격에서 라켓 중심을 공 중심보다 아래에 둘 거리 [m].
pub const IMPACT_CENTER_BELOW_BALL_M: f64 = 0.020;
/// 공을 상대편으로 넘기기 위한 라켓 면의 위쪽 기울기 [deg].
pub const IMPACT_UPWARD_TILT_DEG: f64 = 8.0;
/// 검출 직후 추가 백스윙의 첫 시도 시간 [s].
pub const DETECTION_WINDUP_MIN_DURATION_SECS: f64 = 0.120;
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
