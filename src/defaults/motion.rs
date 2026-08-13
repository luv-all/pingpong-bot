//! 접수 계획 — 인터셉트·bang-bang·Magnus 휴리스틱.

use crate::defaults::rail::RAIL_MOUNT_Y_M;
use crate::robot::motion::InterceptWindow;

/// [`RAIL_MOUNT_Y_M`]에서 인터셉트 구간 하한까지의 고정 오프셋 [m].
///
/// 마운트 실측 -0.128일 때 검증된 하한 0.08(`0.08 - (-0.128)`)로 고정한다 —
/// 마운트가 옮겨져도 팔 기준 도달 가능 범위는 그대로이므로, 월드 y 하한은
/// 마운트를 따라 같은 만큼 이동해야 한다.
pub const INTERCEPT_Y_MIN_OFFSET_FROM_MOUNT_M: f64 = 0.350;
/// [`RAIL_MOUNT_Y_M`]에서 인터셉트 구간 상한까지의 고정 오프셋 [m].
/// 마운트 실측 -0.128일 때 검증된 상한 0.35(`0.35 - (-0.128)`)로 고정한다.
pub const INTERCEPT_Y_MAX_OFFSET_FROM_MOUNT_M: f64 = 0.480;
/// 기본 인터셉트 구간 하한 y [m] — [`RAIL_MOUNT_Y_M`]이 바뀌면 같이 이동한다.
pub const INTERCEPT_Y_MIN_M: f64 = RAIL_MOUNT_Y_M + INTERCEPT_Y_MIN_OFFSET_FROM_MOUNT_M;
/// 기본 인터셉트 구간 상한 y [m] — [`RAIL_MOUNT_Y_M`]이 바뀌면 같이 이동한다.
pub const INTERCEPT_Y_MAX_M: f64 = RAIL_MOUNT_Y_M + INTERCEPT_Y_MAX_OFFSET_FROM_MOUNT_M;

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

/// 타격 직후 준비 자세 복귀(실시간 랠리)는 다음 공을 최대한 빨리 받아야
/// 하므로 한계 내에서 낼 수 있는 최고 속도로 복귀한다.
pub const RALLY_RETURN_SPEED_RATIO: f64 = 1.0;

/// 발사기 반복 시험용 고정 푸시가 한계를 지키며 임팩트를 만들 최소 시간.
pub const FIXED_IMPACT_MIN_DURATION_SECS: f64 = 0.25;
/// 공이 없을 때 기존 임팩트 자세보다 뒤에서 대기할 거리 [m].
///
/// 큰 동작을 줄인 반복 발사기 시험값 2cm.
pub const READY_PREWIND_DISTANCE_M: f64 = 0.020;
/// 공이 없을 때 미리 맞춰 둘 대표 라켓 중심 높이 [m].
///
/// [`crate::defaults::robot::READY_JOINTS_4DOF`]의 FK z를 그대로 쓴다 — 벤치
/// 정렬 자세(홈 포지션)가 재보정되면 준비 높이도 같이 이동해, 둘을 따로
/// 맞출 필요가 없다.
pub fn ready_racket_height_m() -> f64 {
    return crate::defaults::robot::ready_racket_pose().position.z;
}
/// 기본 인터셉트 구간 중앙의 준비 타격 y [m].
///
/// [`crate::defaults::robot::READY_JOINTS_4DOF`]의 FK y를 그대로 쓴다 — 벤치
/// 정렬 자세(홈 포지션)가 재보정되면 준비 타격 y도 같이 이동해, 둘을 따로
/// 맞출 필요가 없다.
pub fn ready_racket_y_m() -> f64 {
    return crate::defaults::robot::ready_racket_pose().position.y;
}
/// 공 검출 후 공 높이에서 유지할 임팩트 자세 기준 백스윙 거리 [m].
/// 준비 자세와 같은 2cm로 두어 검출 뒤 불필요한 추가 감김을 없앤다.
pub const DETECTION_WINDUP_DISTANCE_M: f64 = 0.020;
/// 기본 타격에서 라켓 중심을 공 중심보다 아래에 둘 거리 [m].
pub const IMPACT_CENTER_BELOW_BALL_M: f64 = 0.020;
/// 기초 정렬 모드에서 공이 닿을 지점을 블레이드 중심보다 낮추는 거리 [m].
/// 현재는 블레이드 중심보다 2cm 아래를 맞춘다.
pub const ALIGNMENT_CONTACT_BELOW_RACKET_CENTER_M: f64 = 0.020;
/// 예측된 공 높이에서 실제 정렬 타격점을 올리는 보정 [m].
pub const ALIGNMENT_TARGET_HEIGHT_OFFSET_M: f64 = 0.015;
/// 레일 마운트 X 9.025cm를 기구학에 반영한 뒤에도 남는 타격 X 보정 [m].

/// 2026-08-13 실기 테스트에서 타격점이 목표보다 -X로 밀린다는 보고에 따라,
/// 같은 날 추가했던 실험용 6cm(-X 방향)를 되돌리고 실측 잔여 보정
/// 3.475cm로 복원한다.
pub const ALIGNMENT_TARGET_X_OFFSET_M: f64 = 0.03475;
/// 타격 정렬 시 라켓 면이 수평에서 위로 보는 최소 각도 [deg].
pub const ALIGNMENT_MIN_UPWARD_TILT_DEG: f64 = 25.0;
/// 공을 처음 검출한 뒤 첫 정렬 명령을 허용하기까지 기다리는 시간 [s].
/// 첫 유효 예측에서 즉시 레일·팔을 출발시키고, 정확한 타격 확정은 별도의
/// 연속 안정화 조건이 담당한다.
pub const FIRST_CONTROL_AFTER_DETECTION_SECS: f64 = 0.010;
/// 공을 상대편으로 넘기기 위한 라켓 면의 위쪽 기울기 [deg].
pub const IMPACT_UPWARD_TILT_DEG: f64 = 40.0;
/// 검출 직후 추가 백스윙의 첫 시도 시간 [s].
pub const DETECTION_WINDUP_MIN_DURATION_SECS: f64 = 0.120;
/// 임팩트 순간 목표 라켓 선속도 [m/s].
/// 백스윙 없이 길게 바로 밀 때 사용할 임팩트 목표 선속도.
/// 공이 네트를 넘을 추진력을 확보하도록 0.85m/s에서 1.20m/s로 올렸다.
pub const FIXED_IMPACT_PUSH_SPEED_M_S: f64 = 1.20;
/// 정렬 시 공의 접촉점보다 뒤에서 대기해 다관절 푸시 가속거리를 확보한다 [m].
/// 전역 READY 자세는 실물 캘리브레이션이므로 바꾸지 않고, 공별 타격 준비
/// 자세만 이 거리만큼 접는다.
pub const FIXED_JOINT_PUSH_DISTANCE_M: f64 = 0.100;
/// 직진 푸시 10cm 동안 라켓 중심을 함께 들어 올릴 거리 [m].
/// 아래쪽에서 공을 퍼 올리도록 j2를 상승 추진에 사용한다.
pub const FIXED_JOINT_PUSH_LIFT_M: f64 = 0.020;
/// 백스윙 없는 다관절 직진 푸시 시작부터 예상 타격점에 도달하는 시간 [s].
/// 10cm 전진과 설정된 임팩트 속도를 백스윙 없이 만족하는 궤적 길이다.
pub const FIXED_JOINT_SWING_DURATION_SECS: f64 = 0.200;
/// 예상 공 도착 시각보다 스윙 명령을 앞서 시작할 시간 [s].
/// 궤적 길이는 0.20초로 유지하고 명령만 기존보다 0.20초 앞당긴다.
/// sim의 quadratic 스윙(`FIXED_JOINT_SWING_DURATION_SECS`)과만 짝을 이룬다 —
/// 실기 파워 스윙 경로는 [`FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS`]를 쓴다.
pub const FIXED_JOINT_SWING_LEAD_SECS: f64 = 0.400;
/// 임팩트 순간 다관절 푸시가 사용할 설정상 관절 속도 상한 비율.
/// 설정상 상한 자체가 모터 무부하 최고속의 95%이므로 여기서는 전부 사용한다.
pub const FIXED_JOINT_SNAP_SPEED_RATIO: f64 = 1.0;
/// 임팩트 이후에도 같은 방향으로 계속 밀고 멈추는 시간 [s].
///
/// 이 값을 늘려 [`FIXED_JOINT_SWING_IMPACT_MARGIN_SECS`]와의 격차를 메우려
/// 했으나(2026-08-14 시도), `target_impact_time_secs`가 이미
/// [`FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS`] 하한에 걸린 온스케줄 표준
/// 경우 j0·j2가 임팩트 순간 이미 관절속도 상한 근처라 팔로스루 끝점이
/// 관절한계에 클램프되고, 그 늘어난 팔로스루 시간 동안 그 좁아진 거리를
/// 감속으로 메우려는 quintic이 실현 불가능해져(`evaluate_trajectory_feasibility`
/// 실패) 강제 2cm 비상 폴백으로 떨어지는 회귀를 만들었다(`diag_wp*` 기존
/// 테스트가 이 회귀를 그대로 잡아냈다). 그래서 팔로스루 자체는 원래
/// 값(0.120)으로 유지하고, 격차는 대신
/// [`FIXED_JOINT_SWING_IMPACT_MARGIN_SECS`] 쪽을 줄여 메운다.
pub const FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS: f64 = 0.120;
/// 파워 스윙에서 j0·j2가 정지에서 관절 속도 상한까지 가속하는 데 쓰는
/// 시간 [s]. `arm.max_joint_speed / FIXED_JOINT_SWING_RAMP_SECS`가 이 관절들의
/// 가속도로 쓰인다.
pub const FIXED_JOINT_SWING_RAMP_SECS: f64 = 0.060;
/// 손목(j3)이 등가속 스냅에 쓸 최소 시간 [s] — 실제 스냅 시간은
/// `2·|Δq3|/max_joint_speed`(정지에서 관절 속도 상한까지 걸리는 최소
/// 시간)로 요구 회전량에 맞춰 계산하고, 이 값은 그 계산이 0에 가까운
/// 회전량에도 지나치게 짧은 스냅을 만들지 않게 막는 하한이다.
/// (2026-08-14 이전에는 이 값이 고정 스냅 시간 자체였다 — 임팩트까지
/// j3 요구 회전량이 커질수록(최대 -24°) 50ms 창을 넘어서면서 궤적 전체가
/// 강제로 2cm 비상 폴백까지 떨어지는 문제가 있었다.)
pub const FIXED_JOINT_SWING_MIN_SNAP_SECS: f64 = 0.050;
/// 스냅 창 계산에 쓸 목표 속도를 `max_joint_speed`의 이 비율까지만 노리게
/// 하는 여유 [무차원]. 스냅(등가속) 구간의 끝 속도·가속도는 그대로
/// 팔로스루(quintic, [`FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS`]) 시작
/// 경계조건이 되는데, quintic은 그 경계 가속도가 0이 아니면 끝 지점
/// 직후 속도가 잠깐 더 올라갔다 내려온다(관성처럼) — 스냅 속도를 상한
/// 그대로 겨냥하면 이 오버슈트가 실제 첨두 속도를 한계 밖으로 밀어낸다.
/// 실측(2026-08-14): 스냅 시간이 하한(`FIXED_JOINT_SWING_MIN_SNAP_SECS`)에
/// 걸린 대표 사례들에서 오버슈트가 약 11~15% — 0.85를 곱해 그만큼의 여유를
/// 남긴다.
pub const FIXED_JOINT_SWING_SNAP_VELOCITY_MARGIN: f64 = 0.85;
/// 파워 스윙 타격-전 시간의 하한 [s] — `FIXED_JOINT_SWING_RAMP_SECS`(0.06)
/// + 이전에 고정이었던 순항 시간(0.06)과 같은 값으로, 오늘 출하되는
/// 스윙보다 짧아지지 않도록 막는다. 실제 타격-전 시간은
/// `control_worker`가 예상 도착 시각까지 남은 시간에서
/// [`FIXED_JOINT_SWING_IMPACT_MARGIN_SECS`]를 뺀 값으로 동적으로 계산하고,
/// 이 하한으로 클램프한다.
pub const FIXED_JOINT_SWING_MIN_IMPACT_TIME_SECS: f64 = 0.120;
/// 예상 공 도착 시각보다 몇 초 먼저 임팩트가 나야 하는지 — 파워 스윙의
/// 목표 타격-전 시간을 `남은 시간 − 이 값`으로 계산하는 데 쓴다.
///
/// **2026-08-14 축소(0.200→0.140):** 온스케줄 기준 궤적의 "임팩트"
/// 키프레임은 도착 시각보다 이 마진만큼 앞서 잡히고, 그 뒤
/// [`FIXED_JOINT_SWING_FOLLOW_THROUGH_SECS`](0.120)짜리 팔로스루가 관절속도를
/// 0으로 줄인다 — 마진이 팔로스루보다 크면(과거 0.200 vs 0.120, 격차
/// 80ms) 라켓이 실제 공 도착보다 그 격차만큼 먼저 완전히 멈춰 "멈춘
/// 라켓에 맞는" 문제가 있었다(사용자 보고, 2026-08-14). 팔로스루 쪽을
/// 늘리는 대신(그러면 임팩트 순간 이미 관절한계 근접인 표준 스윙에서
/// 클램프·감속 불가로 실현 불가능해짐 — 위 팔로스루 상수 문서 참고)
/// 마진을 줄여 격차를 80ms→20ms로 좁힌다. 완전히 0으로 맞추지 않고
/// 20ms를 남기는 건 제어 루프 응답 지연·예측 오차를 흡수할 여유를
/// 완전히 없애지 않기 위해서다 — 남은 여유가 이전보다 줄었으므로 실기
/// 재검증 전까지는 보수적으로 남긴 값.
pub const FIXED_JOINT_SWING_IMPACT_MARGIN_SECS: f64 = 0.050;
/// 예상 공 도착 시각보다 파워 스윙 명령을 앞서 시작할 시간 [s] —
/// [`FIXED_JOINT_SWING_LEAD_SECS`]의 파워 스윙 전용 짝. 스윙이 언제
/// 트리거되는지만 정하고, 스윙 자체의 소요 시간은
/// [`FIXED_JOINT_SWING_IMPACT_MARGIN_SECS`]로 별도 계산한다.
pub const FIXED_JOINT_SWING_POWER_SWEEP_LEAD_SECS: f64 = 0.320;
impl Default for InterceptWindow {
    fn default() -> Self {
        // 즉시 출발하되 기존 검증된 접수 범위(마운트 기준 오프셋) 안에서 타격점을 고른다.
        return Self {
            y_min: INTERCEPT_Y_MIN_M,
            y_max: INTERCEPT_Y_MAX_M,
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
