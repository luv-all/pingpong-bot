//! 접수 계획 — 인터셉트·bang-bang·Magnus 휴리스틱.

use crate::robot::motion::InterceptWindow;

/// 인터셉트 샘플 상한.
pub const MAX_INTERCEPT_SAMPLES: usize = 1_024;

/// Magnus |ω| 클립 [rad/s].
pub const MAGNUS_OMEGA_MAX: f64 = 80.0;

pub const POSITION_TOLERANCE_RAD_OR_M: f64 = 1e-3;
pub const RACKET_SPEED_RATIO_TOLERANCE: f64 = 0.15;
pub const RACKET_DIRECTION_TOLERANCE_DEG: f64 = 15.0;
pub const PLAN_DT_SECS: f64 = 0.001;
pub const MAX_PLAN_TIME_SECS: f64 = 0.5;
pub const JACOBIAN_DAMPING: f64 = 0.05;
pub const TIME_TO_GO_BIAS: f64 = 0.5;
pub const MIN_TIME_TO_GO_SECS: f64 = 1e-3;
pub const JDOT_STEP: f64 = 1e-4;

pub const RETURN_TO_CENTER_MIN_SECS: f64 = 0.2;
pub const RETURN_TO_CENTER_MAX_SECS: f64 = 3.0;
pub const RETURN_TO_CENTER_GROWTH: f64 = 1.4;
/// 예측된 공 도착 시각부터 준비 자세 복귀를 시작할 때까지 유지하는 시간.
pub const POST_ALIGNMENT_HOLD_SECS: f64 = 0.5;

/// 모드 1/2/3 홈 포지션 변경·시작 시 센터(ready) 복귀는 랠리처럼 빠를 필요가
/// 없다 — `Planner::move_to_at_speed_ratio`/`return_to_center_at_speed_ratio`로
/// 관절·레일 속도를 이 비율만큼 늦춘다.
pub const HOME_RETURN_SPEED_RATIO: f64 = 1.0 / 3.0;

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
/// 기초 정렬 모드에서 공이 닿을 지점을 블레이드 중심보다 낮추는 거리 [m].
/// 현재는 블레이드 중심보다 0.5cm 아래를 맞춘다.
pub const ALIGNMENT_CONTACT_BELOW_RACKET_CENTER_M: f64 = 0.005;
/// 예측된 공 높이에서 실제 정렬 타격점을 올리는 보정 [m].
pub const ALIGNMENT_TARGET_HEIGHT_OFFSET_M: f64 = 0.015;
/// 타격 정렬 시 라켓 면이 수평에서 위로 보는 최소 각도 [deg].
pub const ALIGNMENT_MIN_UPWARD_TILT_DEG: f64 = 25.0;
/// 발사기 기준 오른쪽으로 적용하는 공별 타격 예측 위치 보정 [m].
/// 현재 실물 레일은 `reverse=true`이므로 오른쪽은 제어 x 감소 방향이다.
pub const ALIGNMENT_LAUNCHER_RIGHT_OFFSET_M: f64 = 0.060;
/// 공을 처음 검출한 뒤 첫 정렬 명령을 허용하기까지 기다리는 시간 [s].
///
/// 너무 이른 단일 관측으로 레일과 팔이 출발하지 않도록 0.15초 동안 관측을
/// 모은 뒤, 그 이후 들어온 첫 유효 예측부터 제어에 사용한다.
pub const FIRST_CONTROL_AFTER_DETECTION_SECS: f64 = 0.150;
/// 공을 상대편으로 넘기기 위한 라켓 면의 위쪽 기울기 [deg].
pub const IMPACT_UPWARD_TILT_DEG: f64 = 8.0;
/// 검출 직후 추가 백스윙의 첫 시도 시간 [s].
pub const DETECTION_WINDUP_MIN_DURATION_SECS: f64 = 0.120;
/// 임팩트 순간 목표 라켓 선속도 [m/s].
/// 백스윙 없이 길게 바로 밀 때 사용할 임팩트 목표 선속도.
pub const FIXED_IMPACT_PUSH_SPEED_M_S: f64 = 0.70;
/// 정렬 시 공의 접촉점보다 뒤에서 대기해 다관절 푸시 가속거리를 확보한다 [m].
/// 전역 READY 자세는 실물 캘리브레이션이므로 바꾸지 않고, 공별 타격 준비
/// 자세만 이 거리만큼 접는다.
pub const FIXED_JOINT_PUSH_DISTANCE_M: f64 = 0.100;
/// 백스윙 없는 다관절 직진 푸시 시작부터 예상 타격점에 도달하는 시간 [s].
/// 뒤로 거리를 벌리지 않고 현재 자세에서 바로 미는 짧은 타격 구간.
pub const FIXED_JOINT_SWING_DURATION_SECS: f64 = 0.250;
/// 임팩트 순간 다관절 푸시가 사용할 설정상 관절 속도 상한 비율.
/// 설정상 상한 자체가 모터 무부하 최고속의 95%이므로 여기서는 전부 사용한다.
pub const FIXED_JOINT_SNAP_SPEED_RATIO: f64 = 1.0;
/// 임팩트 이후에도 같은 방향으로 계속 밀고 멈추는 시간 [s].
pub const FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS: f64 = 0.120;
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
