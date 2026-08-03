//! 접수 계획 — 인터셉트·bang-bang·Magnus 휴리스틱.

use crate::robot::motion::InterceptWindow;

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
        // 공 궤적에서 먼저 넓은 후보를 만들고, 각 후보의 레일+팔 IK·
        // 속도·가속·토크 실현 가능성으로 걸러낸다. 이 창은 "다 친다"가
        // 아니라 "IK를 시도할 공 후보 공간"이다. 예전 [0.08, 0.35]m는
        // 로봇 코앞의 27cm만 보아 타격 기회를 너무 일찍 버렸다. 테이블
        // 충돌·도달 불가는 하류 플래너가 판정하므로 로봇 끝선 0cm부터
        // 테이블 안쪽 55cm까지를 검색한다.
        return Self {
            y_min: 0.00,
            y_max: 0.55,
            sample_step: 0.025,
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
