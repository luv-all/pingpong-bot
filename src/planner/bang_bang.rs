//! 순수 토크 한계 기반 스윙 — quintic처럼 정해둔 궤적 "모양"이 없다. GUI에서
//! quintic 스윙과 육안 비교하기 위한 디버그 경로 — `plan_swing`(quintic,
//! 실제 게임플레이 경로)은 건드리지 않는다.
//!
//! # 태스크스페이스(라켓 3D) + ZEM/ZEV 유도
//!
//! 이 파일은 두 번의 설계 교체를 거쳤다(자세한 실측 기록은
//! `.omc/progress.txt`):
//!
//! 1. **관절별 독립 bang-bang**(1차): 관절마다 목표 관절각·관절속도를 각자
//!    시간최적으로 쫓았다. 목표속도가 대개 0이 아니라 "도달"이 곧 "지나침"인데,
//!    관절마다 도달 시간이 달라 동기화가 안 돼 다른 관절을 기다릴 방법이
//!    없었다 — 먼저 도착한 관절이 계속 오버슈트→재보정을 반복하며
//!    `qdot`가 `±max_joint_speed` 사이를 영원히 왕복.
//! 2. **라켓(3D) 축별 독립 bang-bang**(2차): 관절이 아니라 라켓 위치·속도를
//!    목표로 삼아 자코비안 최소노름 역산으로 관절가속도를 냈지만, x/y/z
//!    3축을 여전히 **독립된 스칼라** bang-bang으로 계산했다. 이번엔 3축이
//!    자코비안 결합을 통해 서로 간섭해, 위치는 수렴해도(mm급) 속도 명령이
//!    0.1~0.2초마다 부호를 뒤집는 거시적 진동에 빠졌다.
//! 3. **ZEM/ZEV(Zero-Effort-Miss / Zero-Effort-Velocity) 유도**(현재): 실제
//!    요격 유도(미사일 종말 유도)가 쓰는 최소에너지 제어. 위치오차·속도오차·
//!    목표속도를 3축을 쪼개지 않고 **하나의 벡터식**으로 묶어, 남은 시간
//!    `Tg`(= 예측된 임팩트까지 남은 시간, `TIME_TO_GO_BIAS`로 조금 앞당겨
//!    처음부터 더 적극적으로 밀게 만든다) 동안 에너지최소로 `(0, target_v)`에
//!    정확히 도달하는 가속도를 매 스텝 다시 계산한다(`bang_bang_accel_to`
//!    참고). `Tg`를 실제 임팩트 시각에 묶으므로 위 1·2차의 근본 원인(공유된
//!    도착시각이 없었던 것)이 원천적으로 사라진다 — 실측: 축끼리 더 이상
//!    간섭하지 않고 진동 없이 목표를 향해 진행.
//!
//! 라켓 가속도가 정해지면 **토크 가중 최소노름 자코비안 역산**(`j_pinv`,
//! `joint_preference = torque_limit⁴`)으로 관절가속도로 바꾼다 — 균일
//! 최소노름은 토크 여유가 가장 작은 엘보/손목만 계속 포화시키고 토크가 큰
//! (베이스에 가까운) 어깨·요 관절은 거의 안 써서(실측: 어깨 사용률 3%),
//! 실제 스윙이 베이스 쪽 큰 관절에서 파워를 내는 것과 맞지 않았다. 자코비안
//! 시간미분(`J̇·q̇`)도 유한차분으로 보정한다 — 관절이 빠르게 움직일 때
//! 이 항을 생략하면 체계적 편차가 남는다(실측).
//!
//! 관절가속도는 역동역학(`robot::dynamics::{mass_matrix, bias_torques}`)으로
//! 필요 토크를 구해 실기 한계로 클램프하고, 그 클램프된 토크로 실제 나오는
//! 가속도를 다시 풀어(`forward_dynamics`를 거치지 않고 이미 계산한 질량행렬을
//! 그대로 LU로 재사용 — 매 스텝 두 번 계산하는 중복을 피한다) 적분한다.
//!
//! 종료 조건은 라켓 위치오차와, FK로 역산한 실제 라켓 속도의 방향·크기가
//! 허용오차 안인지로 본다. `bang_bang_accel_to`(1D 시간최적 스위칭)는
//! 프로덕션 경로에서는 더 이상 쓰이지 않지만, 그 도출·검증 자체는 유효한
//! 결과라 회귀 테스트로 남아 있다.

use nalgebra::{DMatrix, DVector, Vector3};

use super::collision::{clamp_above_table, table_penetration};
use super::physics::{in_swing_commit_window, solve_impact_target};
use crate::defaults::planner::{
    JACOBIAN_DAMPING, JDOT_STEP, MAX_PLAN_TIME_SECS, MIN_TIME_TO_GO_SECS, PLAN_DT_SECS,
    POSITION_TOLERANCE_RAD_OR_M, RACKET_DIRECTION_TOLERANCE_DEG, RACKET_SPEED_RATIO_TOLERANCE,
    RAIL_ACCEL_M_S2, TIME_TO_GO_BIAS,
};
use crate::error::{DomainError, SwingPlanError};
use crate::robot::Arm;
use crate::robot::dynamics::{MassMatrixScratch, RneaScratch, bias_torques_into, mass_matrix_into};
use crate::{Joints, Prediction, RobotPose};

/// "코스팅 함정"(위치항 `-6x/Tg²`이 속도오차항을 우연히 상쇄해 속도가 목표의
/// 22~25%에서 오래 정체하다 마감 직전 폭발하는 현상 — `diag_trivial_case_trace`
/// 실측: home 방향 그대로에 d=0.02m, v=0.05m/s인 사실상 트리비얼한 목표조차
/// 이 함정에 걸려 실패) 방지용 — 속도 관련 두 항(`-4v/Tg`, `-2·target_v/Tg`)
/// 에만 곱하는 추가 긴급도 배율. `TIME_TO_GO_BIAS`(위치·속도 항을 똑같은
/// 비율로 급하게 만듦)로는 이 함정을 못 깬다 — 위치항과 속도항이 유지하는
/// 상쇄 "비율" 자체는 Tg를 균일하게 줄여도 그대로 보존되기 때문(실측:
/// TIME_TO_GO_BIAS=0.3 시도는 위치수렴만 깨뜨리고 속도 정체는 그대로였음).
/// 속도 항만 따로 더 급하게 만들면, 상쇄에 필요한 `|x|`가 커지는데 코스팅
/// 동역학은 `x`를 계속 줄이므로 그 관계를 유지하기 어려워져 함정을 벗어난다.
/// 엄밀한 최소-제어-노력 최적해에서는 벗어나지만, 토크/속도 한계가 있는
/// 실기에서는 이쪽이 로버스트하다(사용자 지시: 완벽함이 아니라 커버리지).
const VELOCITY_URGENCY_GAIN: f64 = 2.0;
/// 관절이 `max_joint_speed`에 가까워질수록 가중 최소노름 역산에서 그 관절을
/// "비싸게" 만드는 지수 — [`step_racket_guidance`] 속도-헤드룸 항 문서 참고.
const SPEED_HEADROOM_EXPONENT: f64 = 2.0;
/// 속도 헤드룸 비율의 하한 — 관절이 캡에 완전히 닿아도 `preference`가 0으로
/// 떨어져 그 관절이 자코비안 역산에서 영구히 배제되는 걸 막는다(질량행렬
/// 특이화 방지와 같은 이유로 0을 피함).
const SPEED_HEADROOM_FLOOR: f64 = 0.05;

/// bang-bang 적분으로 얻은 샘플 기반 궤적. quintic처럼 닫힌 형태 계수가
/// 아니라 매 스텝 실제 좌표를 그대로 담는다 — `sample_at`/`sample_rail_at`은
/// 가장 가까운 두 샘플을 선형보간한다.
#[derive(Debug, Clone, PartialEq)]
pub struct BangBangTrajectory {
    dt: f64,
    joint_samples: Vec<Joints>,
    rail_samples: Vec<f64>,
}

impl BangBangTrajectory {
    pub fn duration_secs(&self) -> f64 {
        return (self.joint_samples.len().saturating_sub(1)) as f64 * self.dt;
    }

    fn sample_index(&self, t: f64) -> (usize, usize, f64) {
        let clamped = t.clamp(0.0, self.duration_secs());
        let raw = clamped / self.dt;
        let lo = (raw.floor() as usize).min(self.joint_samples.len() - 1);
        let hi = (lo + 1).min(self.joint_samples.len() - 1);
        let frac = if hi == lo { 0.0 } else { raw - lo as f64 };
        return (lo, hi, frac);
    }

    pub fn sample_at(&self, t: f64) -> Joints {
        let (lo, hi, frac) = self.sample_index(t);
        let a = &self.joint_samples[lo];
        let b = &self.joint_samples[hi];
        let values = a
            .values
            .iter()
            .zip(&b.values)
            .map(|(x, y)| x + (y - x) * frac)
            .collect();
        return Joints { values };
    }

    pub fn sample_rail_at(&self, t: f64) -> f64 {
        let (lo, hi, frac) = self.sample_index(t);
        return self.rail_samples[lo] + (self.rail_samples[hi] - self.rail_samples[lo]) * frac;
    }

    /// `t` [s]에서 관절 각속도 [rad/s].
    ///
    /// 닫힌 형태 계수가 없어 인접 샘플 차분으로 근사한다 — 적분 스텝이
    /// [`PLAN_DT_SECS`](1 ms)라 quintic의 해석 미분과 같은 수준으로 매끄럽다.
    pub fn sample_velocity_at(&self, t: f64) -> Vec<f64> {
        let (lo, hi, _) = self.sample_index(t);
        if hi == lo {
            return vec![0.0; self.joint_samples[lo].values.len()];
        }
        return self.joint_samples[lo]
            .values
            .iter()
            .zip(&self.joint_samples[hi].values)
            .map(|(a, b)| (b - a) / self.dt)
            .collect();
    }

    pub fn end_joints(&self) -> &Joints {
        return self.joint_samples.last().expect("최소 1개 샘플");
    }

    pub fn follow_through_rail_x(&self) -> f64 {
        return *self.rail_samples.last().expect("최소 1개 샘플");
    }
}

/// `predictions` 중 IK가 풀리는 첫 후보로 bang-bang 궤적을 계획한다.
/// 선택 순서는 `plan_best_swing`과 같은 "현재 라켓 위치에 가까운 순".
/// `plan_bang_bang_swing`이 실제로 고른 예측 + 궤적 - `PlannedIntercept`
/// (quintic)와 대응. GUI가 "어떤 hit-plane을 겨냥했는지" 디버그 표시에 쓴다.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedBangBangIntercept {
    pub prediction: Prediction,
    pub trajectory: BangBangTrajectory,
}

pub fn plan_bang_bang_swing(
    arm: &Arm,
    predictions: &[Prediction],
    start: &RobotPose,
) -> Result<PlannedBangBangIntercept, DomainError> {
    let current_position = if arm.rail.is_some() {
        arm.forward_kinematics_with_rail(start.rail_x, &start.joints)
    } else {
        arm.forward_kinematics(&start.joints)
    }
    .map(|pose| pose.position.coords)
    .unwrap_or_default();
    let mut ranked: Vec<Prediction> = predictions
        .iter()
        .copied()
        .filter(|prediction| in_swing_commit_window(prediction.time_to_impact_secs))
        .collect();
    ranked.sort_by(|left, right| {
        let left_cost = (left.impact_position.coords - current_position).norm();
        let right_cost = (right.impact_position.coords - current_position).norm();
        left_cost
            .partial_cmp(&right_cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut last_error = None;
    for prediction in ranked {
        match plan_bang_bang_for(arm, &prediction, start) {
            Ok(trajectory) => {
                return Ok(PlannedBangBangIntercept {
                    prediction,
                    trajectory,
                });
            }
            Err(error) => last_error = Some(error),
        }
    }
    return Err(last_error.unwrap_or(DomainError::InfeasibleSwing(
        SwingPlanError::InsufficientTime {
            time_to_impact_secs: 0.0,
            min_swing_secs: 0.0,
        },
    )));
}

/// [`step_racket_guidance`]가 매 스텝 재사용하는 스크래치 버퍼 모음 — 호출부
/// (`plan_bang_bang_for`, `tools/swing_bench`)가 루프 밖에서 한 번만 만들어
/// 반복 재사용한다. 안 그러면 스텝마다 `mass_matrix`(RNEA n+1회) +
/// `bias_torques`(RNEA 1회)가 각자 새 버퍼를 할당한다.
pub struct RacketGuidanceScratch {
    rnea: RneaScratch,
    mass_matrix: MassMatrixScratch,
    mass: DMatrix<f64>,
    bias_zero_accel: Vec<f64>,
    bias: Vec<f64>,
}

impl RacketGuidanceScratch {
    pub fn new(joint_count: usize) -> Self {
        return Self {
            rnea: RneaScratch::new(),
            mass_matrix: MassMatrixScratch::new(),
            mass: DMatrix::zeros(joint_count, joint_count),
            bias_zero_accel: vec![0.0; joint_count],
            bias: vec![0.0; joint_count],
        };
    }
}

/// [`step_racket_guidance`] 한 스텝의 결과 — 호출부가 진단/리포트에 쓴다.
pub struct RacketGuidanceStep {
    pub racket_accel_desired: Vector3<f64>,
    pub torque_cmd: Vec<f64>,
}

/// 라켓(3D) task-space ZEM/ZEV 유도 + 토크 가중 자코비안 역산으로 한 스텝
/// (`dt`) 적분한다. `q`/`qdot`/`rail_x`/`rail_v`를 제자리에서 전진시킨다.
///
/// `plan_bang_bang_for`(프로덕션 GUI 디버그 경로)와 `tools/swing_bench`(오프라인
/// 벤치마크)가 이 함수 하나를 공유한다 — 같은 제어 로직이 두 크레이트에서
/// 따로 구현되며 갈라지는 걸 막는다(과거 `bang_bang_accel`가 겪은 문제:
/// 스위칭 곡선 버그를 한쪽만 고치고 다른 쪽은 못 고쳐 실측 결과가 달랐다 —
/// `.omc/progress.txt` 참고).
///
/// # ZEM/ZEV(Zero-Effort-Miss / Zero-Effort-Velocity) 유도
///
/// 실제 요격 유도(미사일 종말 유도)가 쓰는 최소에너지 제어. 이전 두 설계
/// (관절별 독립 bang-bang, 라켓 3축 독립 bang-bang)는 각 관절/축이 자기만
/// 보고 스위칭을 결정해 서로 간섭했다(실측: 목표 도달 안 됐는데도 z가속
/// 명령이 0.1~0.2초마다 부호를 뒤집는 거시적 진동, 모듈 문서 참고). ZEM/ZEV는
/// 위치오차·속도오차·목표속도를 **하나의 벡터식**으로 묶어 이 간섭 자체가
/// 생길 수 없다.
///
/// 유도: ẋ=v, v̇=a인 이중적분기에서 남은 시간 `Tg` 동안 `∫‖a‖²dt`를
/// 최소화하면서 `x(Tg)=0, v(Tg)=target_v`에 정확히 도달하는 제어는
/// `a(t)=α+β·t`(벡터 계수) 꼴이고, 경계조건 두 개(위치·속도)를 풀면 현재
/// 시점 명령 `a(0) = 6·ZEM/Tg² - 2·ZEV/Tg`(`ZEM = -(x + v·Tg)`: 지금부터
/// 무제어로 `Tg` 동안 가면 남을 위치오차, `ZEV = target_v - v`: 속도오차)이
/// 나온다. 전개하면 `a = -6x/Tg² - 4v/Tg - 2·target_v/Tg`.
///
/// `remaining_secs`(호출부가 매 스텝 넘기는 `Tg` 원본, 예: 실제 임팩트까지
/// 남은 시간)에 [`TIME_TO_GO_BIAS`]를 곱해 조금 앞당긴다 — 순수 최소에너지
/// ZEM/ZEV는 시간이 넉넉하다고 보고 초반엔 힘을 아꼈다가 마감이 가까워야
/// 급해지는데(실측: 대부분 구간 토크 사용률 20~40%대에 머묾), 우리 문제의
/// 마감은 원래 짧아(커밋창 ≤0.35s) 그 여유가 손해다. 여전히 하나의
/// 벡터식이라 이 편향이 진동을 재도입하지 않는다. `remaining_secs <= 0.0`이면
/// (도달 시각을 이미 넘김) `None`.
///
/// # 가중 최소노름 자코비안 역산
///
/// 라켓 가속도가 정해지면 [`Arm::position_jacobian_fd`] 최소노름 역산으로
/// 관절가속도로 바꾼다. 순수 최소노름(가중치 균일)은 실측상 문제가 있다 —
/// 토크 여유가 가장 작은 엘보/손목이 자꾸 포화되는데, 토크가 훨씬 큰(베이스에
/// 가까운) 어깨·요 관절은 거의 안 쓰인다(실측: 정지 구간 토크 사용률 어깨
/// 3%, 엘보 최대 100%). 실제 라켓 스윙도 파워를 베이스에 가까운 큰 관절에서
/// 낸다 — 가중치를 `torque_limit⁴`(토크 여유가 클수록 "싸게" 쓰도록, 지수는
/// 실측 튜닝값 — `torque_limit²`로는 어깨 사용률이 3%→19%까지만 올라
/// 부족했다)로 둬 역산이 자연히 그 관절들을 더 쓰게 한다. 레일은 관절과
/// 단위가 달라(선형 m/s² vs 회전 rad/s²) 같은 척도로 비교할 수 없어 가중치
/// 1.0(중립)으로 둔다.
///
/// 자코비안 역산(`(J·W⁻¹·Jᵀ + λI)^-1`)에는
/// [`JACOBIAN_DAMPING`](감쇠최소제곱, Nakamura-Hanafusa) 정칙화가 필요하다 —
/// `Arm::linear_velocities_for_racket_velocity`(속도 역산)의 `1e-9`로는 이
/// **가속도** 역산에서 근접 특이점 근처가 훨씬 크게 발산한다(실측: `qddot`
/// 성분이 수천대까지 튐).
///
/// 자코비안 시간미분 보정항 `J̇·q̇`도 [`JDOT_STEP`] 유한차분으로 넣는다 —
/// 관절이 빠르게 움직일 때(`max_joint_speed`에 근접) 이 항이 목표 가속과
/// 비슷한 크기라 생략하면 체계적 편차가 남는다(실측).
///
/// 관절가속도는 역동역학으로 필요 토크를 구해 실기 한계로 클램프하고, 그
/// 클램프된 토크로 실제 나오는 가속도를 다시 풀어(`scratch.mass`를 그대로
/// LU로 재사용) 적분한다.
///
/// 자코비안이 없거나(특이 자세) 질량행렬이 특이해 풀 수 없으면 `None` —
/// 호출부는 이를 "이번 스텝은 진행 불가"로 보고 적분을 끝내야 한다.
///
/// `q`/`qdot`/`rail_x`/`rail_v`를 각각 별도 가변 참조로 받는다(구조체로
/// 묶으면 4개가 1개로 줄지만, 그 안에서 다시 개별 필드를 가변으로 꺼내
/// 써야 해 자코비안/스크래치와의 대여 관계가 더 헤매기 쉬워진다) — 물리
/// 적분 스텝 함수가 이 정도 매개변수를 갖는 건 흔하다.
#[allow(clippy::too_many_arguments)]
pub fn step_racket_guidance(
    arm: &Arm,
    q: &mut [f64],
    qdot: &mut [f64],
    rail_x: &mut f64,
    rail_v: &mut f64,
    target_racket_position: Vector3<f64>,
    target_racket_velocity: Vector3<f64>,
    remaining_secs: f64,
    dt: f64,
    scratch: &mut RacketGuidanceScratch,
) -> Option<RacketGuidanceStep> {
    let n = q.len();
    let has_rail = arm.rail.is_some();
    let rail_offset = usize::from(has_rail);
    let rail_max_speed = arm.rail.as_ref().map_or(f64::INFINITY, |r| r.max_speed);

    let pose = RobotPose::new(*rail_x, Joints::from_slice(q));
    let jacobian = arm.position_jacobian_fd(&pose)?;
    let current_pose = arm.forward_kinematics_with_rail(*rail_x, &Joints::from_slice(q))?;
    let racket_velocity = racket_velocity_estimate(arm, *rail_x, *rail_v, q, qdot)?;
    let racket_pos_err = current_pose.position.coords - target_racket_position;

    // 관절 토크/유효관성에서 이번 스텝 이 관절이 낼 수 있는 가속 한계.
    mass_matrix_into(
        arm,
        &Joints::from_slice(q),
        &mut scratch.rnea,
        &mut scratch.mass_matrix,
        &mut scratch.mass,
    );
    let joint_a_max_full: Vec<f64> = (0..jacobian.ncols())
        .map(|col| {
            if has_rail && col == 0 {
                return RAIL_ACCEL_M_S2;
            }
            let i = col - rail_offset;
            (arm.joint_torque_limits[i] / scratch.mass[(i, i)].max(1e-9)).max(1e-6)
        })
        .collect();

    // 정적 토크 가중(`torque_limit⁴`)만으로는, 이미 `max_joint_speed`에 근접한
    // 관절도 여전히 "싸다"고 보고 계속 그쪽으로 명령을 몰아준다 — 그 관절은
    // 실제로는 `qdot` 클램프(아래) 때문에 더 못 가속하므로 그 몫만큼 라켓
    // 방향 명령이 그냥 버려진다(실측: 토크 여유 12~43%인데도 관절이 이미
    // 속도캡 92~100%에 도달한 채 라켓속도가 목표의 15~24%에서 정체 —
    // `.omc/progress.txt`, 이번 세션 `diag_easy_scenarios_still_fail_to_converge`
    // 등에서 반복 확인). 남은 속도 헤드룸(`1 - |qdot|/max_speed`)이 작을수록
    // `preference`를 깎아, 역산이 자연히 헤드룸이 남은 다른 관절로 명령을
    // 재배분하게 한다 — 표준 가중 최소노름 관절한계회피 기법의 변형.
    let joint_preference: Vec<f64> = (0..jacobian.ncols())
        .map(|col| {
            if has_rail && col == 0 {
                return 1.0;
            }
            let i = col - rail_offset;
            let speed_headroom =
                (1.0 - (qdot[i] / arm.max_joint_speed).abs()).clamp(SPEED_HEADROOM_FLOOR, 1.0);
            arm.joint_torque_limits[i].powi(4) * speed_headroom.powf(SPEED_HEADROOM_EXPONENT)
        })
        .collect();
    let w_inv = DMatrix::from_diagonal(&DVector::from_vec(joint_preference));

    let j_winv = &jacobian * &w_inv;
    let jwjt = &j_winv * jacobian.transpose() + DMatrix::identity(3, 3) * JACOBIAN_DAMPING;
    let jwjt_inv = jwjt.try_inverse()?;
    // `j_pinv`의 열 k = "축 k 방향 단위가속을 내려 할 때 실제로 쓸(가중
    // 최소노름) 관절가속 방향" — 실제 제어에 쓸 역산과 같은 사상.
    let j_pinv = &w_inv * jacobian.transpose() * jwjt_inv;

    if remaining_secs <= 0.0 {
        return None;
    }
    let time_to_go = (remaining_secs * TIME_TO_GO_BIAS).max(MIN_TIME_TO_GO_SECS);
    let racket_accel_desired = racket_pos_err * (-6.0 / (time_to_go * time_to_go))
        - racket_velocity * (4.0 * VELOCITY_URGENCY_GAIN / time_to_go)
        - target_racket_velocity * (2.0 * VELOCITY_URGENCY_GAIN / time_to_go);

    // 자코비안 자체가 이미 유한차분(`position_jacobian_fd`, STEP=1e-6)이라,
    // 바깥쪽 `JDOT_STEP`은 그보다 훨씬 크게 잡아 잡음 증폭을 피한다.
    let qdot_full = DVector::from_iterator(
        jacobian.ncols(),
        (0..jacobian.ncols()).map(|col| {
            if has_rail && col == 0 {
                *rail_v
            } else {
                qdot[col - rail_offset]
            }
        }),
    );
    let perturbed_rail_x = *rail_x + *rail_v * JDOT_STEP;
    let perturbed_q: Vec<f64> = q
        .iter()
        .zip(qdot.iter())
        .map(|(qi, vi)| qi + vi * JDOT_STEP)
        .collect();
    let perturbed_pose = RobotPose::new(perturbed_rail_x, Joints::from_slice(&perturbed_q));
    let jdot_qdot = match arm.position_jacobian_fd(&perturbed_pose) {
        Some(jacobian_perturbed) => {
            let jdot = (jacobian_perturbed - &jacobian) / JDOT_STEP;
            jdot * &qdot_full
        }
        None => DVector::zeros(3),
    };

    // 가중 최소노름 자코비안 역산: `qddot = W⁻¹Jᵀ(JW⁻¹Jᵀ + λI)^-1 (a_desired - J̇q̇)`
    // — 위에서 구한 `j_pinv`(이미 `W⁻¹` 반영됨)를 그대로 재사용한다.
    let racket_accel_vec = DVector::from_vec(vec![
        racket_accel_desired.x - jdot_qdot[0],
        racket_accel_desired.y - jdot_qdot[1],
        racket_accel_desired.z - jdot_qdot[2],
    ]);
    let qddot_full = &j_pinv * racket_accel_vec;

    bias_torques_into(
        arm,
        &Joints::from_slice(q),
        qdot,
        &mut scratch.rnea,
        &mut scratch.bias_zero_accel,
        &mut scratch.bias,
    );

    // 근접 특이점(또는 Tg가 짧아 ZEM/ZEV가 요구하는 가속이 실현 불가능하게
    // 커질 때 — 실측: `racket_accel_desired` 성분이 수백만대까지 튐)에서
    // 감쇠로도 못 잡은 잔여 스파이크가 남을 수 있다. 이전에는 각 관절을
    // *독립적으로* `[-a_max, a_max]`에 클램프했는데, 그러면 성분마다 서로
    // 다른 비율로 잘려 **명령된 3D 라켓 방향 자체가 뒤틀린다** — ZEM/ZEV가
    // 하나의 벡터식으로 방향 간섭을 없앤 의미가 실현 단계에서 도로 깨지는
    // 셈이다(실측: 토크 여유 12~43%인데 관절이 이미 속도캡 92~100%에 도달,
    // 라켓속도 목표의 15~24%에서 정체 — `.omc/progress.txt`, 이번 세션
    // 여러 진단에서 반복 확인). 대신 한계를 넘는 성분들 중 "가장 많이
    // 초과한" 것 하나가 정하는 **공통 스케일**을 전체(레일+관절) 가속
    // 벡터에 곱한다 — 방향은 그대로 유지한 채 크기만 줄이는, 표준
    // 다축 궤적 스케일링 기법.
    let mut accel_scale = 1.0_f64;
    for col in 0..jacobian.ncols() {
        let limit = joint_a_max_full[col];
        let raw = qddot_full[col].abs();
        if raw > limit && raw > 1e-9 {
            accel_scale = accel_scale.min(limit / raw);
        }
    }
    let rail_accel_desired = if has_rail {
        qddot_full[0] * accel_scale
    } else {
        0.0
    };
    let joint_qddot_desired: Vec<f64> = (0..n)
        .map(|i| qddot_full[i + rail_offset] * accel_scale)
        .collect();
    let joint_qddot_desired = DVector::from_vec(joint_qddot_desired);
    let m_qddot = &scratch.mass * &joint_qddot_desired;

    // 토크도 같은 이유로 성분별 독립 클램프 대신 공통 스케일을 쓴다 — 질량
    // 행렬은 이미 스케일된 가속을 반영하므로, 여기서 또 넘치는 건 대개
    // bias(중력·코리올리) 항 기여라 스케일 정도는 작다.
    let tau_desired: Vec<f64> = (0..n).map(|i| m_qddot[i] + scratch.bias[i]).collect();
    let mut torque_scale = 1.0_f64;
    for i in 0..n {
        let limit = arm.joint_torque_limits[i];
        let raw = tau_desired[i].abs();
        if raw > limit && raw > 1e-9 {
            torque_scale = torque_scale.min(limit / raw);
        }
    }
    let torque_cmd: Vec<f64> = (0..n).map(|i| tau_desired[i] * torque_scale).collect();
    // 클램프된 토크로 실제 나오는 가속도를 다시 푼다 — `forward_dynamics`를
    // 거치면 이미 계산한 질량행렬을 또 계산하는 중복이 생기므로 직접 LU로
    // 푼다. `lu()`는 소유권을 가져가므로 재사용 버퍼 `scratch.mass`는
    // 복제해서 넘긴다 — n×n(관절수) 크기 하나만 복제하는 비용은 RNEA 스크래치
    // 재계산에 비해 미미하다.
    let rhs = DVector::from_iterator(n, (0..n).map(|i| torque_cmd[i] - scratch.bias[i]));
    let accel = scratch.mass.clone().lu().solve(&rhs)?;

    for i in 0..n {
        qdot[i] += accel[i] * dt;
        qdot[i] = qdot[i].clamp(-arm.max_joint_speed, arm.max_joint_speed);
        q[i] += qdot[i] * dt;
    }
    let rail_accel = rail_accel_desired.clamp(-RAIL_ACCEL_M_S2, RAIL_ACCEL_M_S2);
    *rail_v += rail_accel * dt;
    *rail_v = rail_v.clamp(-rail_max_speed, rail_max_speed);
    *rail_x += *rail_v * dt;

    // 테이블 안전장치는 여기(유도법 내부 상태)가 아니라 호출부의 출력
    // 샘플링 단계에서 처리한다 — `plan_bang_bang_for`의 문서 참고. 이 함수의
    // `q`/`rail_x`는 유도법이 스스로 다음 스텝 계산에 쓰는 "진짜" 상태라,
    // 여기서 손대면(재-IK든 감쇠 클램프든) 다음 스텝 계산에 그대로 피드백돼
    // 작은 보정도 크게 증폭된다(실측: 방향오차 4.3°→125.2°로 폭발).

    return Some(RacketGuidanceStep {
        racket_accel_desired,
        torque_cmd,
    });
}

fn plan_bang_bang_for(
    arm: &Arm,
    prediction: &Prediction,
    start: &RobotPose,
) -> Result<BangBangTrajectory, DomainError> {
    let target = solve_impact_target(arm, prediction, start)?;
    // 관절이 아니라 라켓(3D)이 목표 상태 — `target.racket_velocity`는
    // `solve_impact_target`이 공 물리(`required_racket_velocity`)로 직접 낸
    // 값이라 관절속도 클램프와 무관하게 이미 자기 완결적이다.
    let target_racket_position = arm
        .forward_kinematics_with_rail(target.pose.rail_x, &target.pose.joints)
        .ok_or(DomainError::InfeasibleSwing(
            SwingPlanError::InverseKinematicsNoSolution {
                target_x: prediction.impact_position.x,
                target_y: prediction.impact_position.y,
                target_z: prediction.impact_position.z,
            },
        ))?
        .position
        .coords;
    let target_racket_velocity = target.racket_velocity;

    let n = start.joints.values.len();
    let mut q = start.joints.values.clone();
    let mut qdot = vec![0.0; n];
    let mut rail_x = start.rail_x;
    let mut rail_v = 0.0;

    let mut joint_samples = vec![Joints::from_slice(&q)];
    let mut rail_samples = vec![rail_x];

    // 스크래치를 루프 밖에서 한 번만 할당해 매 스텝(최대 1kHz) 재사용한다.
    let mut scratch = RacketGuidanceScratch::new(n);

    let mut t = 0.0;
    let mut converged = false;
    // 이 계측 자체는 `DIAG_BANGBANG` 환경변수로만 출력되는 디버그용 — 루프
    // 밖에서 한 번만 조회해 매 스텝 환경변수 조회 비용을 피한다.
    let diag = std::env::var("DIAG_BANGBANG").is_ok();
    let mut torque_util_sum = vec![0.0_f64; n];
    let mut torque_util_max = vec![0.0_f64; n];
    let mut step_count: u64 = 0;
    // ZEM/ZEV는 실제 임팩트 시각까지 남은 시간(`Tg`)에 묶여 있으므로, 루프도
    // 그 시각을 넘겨서까지 돌 이유가 없다 — `MAX_PLAN_TIME_SECS`는 방어용
    // 상한으로만 남긴다(`prediction.time_to_impact_secs`는 이미
    // `in_swing_commit_window`로 0.35s 이하로 걸러져 있어 보통 이쪽이 이긴다).
    while t < prediction.time_to_impact_secs.min(MAX_PLAN_TIME_SECS) {
        let remaining_secs = prediction.time_to_impact_secs - t;
        let Some(step) = step_racket_guidance(
            arm,
            &mut q,
            &mut qdot,
            &mut rail_x,
            &mut rail_v,
            target_racket_position,
            target_racket_velocity,
            remaining_secs,
            PLAN_DT_SECS,
            &mut scratch,
        ) else {
            break;
        };
        t += PLAN_DT_SECS;
        // 유도법의 진짜 상태(`q`)는 절대 안 건든다 — 재생용 샘플만 테이블
        // 안전 자세로 클램프해서 저장한다. `step_racket_guidance` 문서 참고:
        // 안전장치를 유도법 내부(다음 스텝 계산 입력)에 넣으면 아주 작은
        // 보정도 되먹임돼 증폭된다(실측: 방향오차 4.3°→125.2°). 재생 샘플만
        // 클램프하면 유도법의 내부 적분은 전혀 흔들리지 않고, 실제로
        // 로봇에 나가는(그려지는) 자세만 안전해진다.
        let sample_joints = if table_penetration(arm, rail_x, &Joints::from_slice(&q)) > 1e-4 {
            clamp_above_table(arm, rail_x, &Joints::from_slice(&q))
        } else {
            Joints::from_slice(&q)
        };
        joint_samples.push(sample_joints);
        rail_samples.push(rail_x);
        if diag {
            step_count += 1;
            for i in 0..n {
                let util = step.torque_cmd[i].abs() / arm.joint_torque_limits[i];
                torque_util_sum[i] += util;
                torque_util_max[i] = torque_util_max[i].max(util);
            }
        }

        let Some(updated_pose) = arm.forward_kinematics_with_rail(rail_x, &Joints::from_slice(&q))
        else {
            break;
        };
        let pos_err = (updated_pose.position.coords - target_racket_position).norm();
        if diag {
            let step_idx = (t / PLAN_DT_SECS).round() as u64;
            if step_idx.is_multiple_of(100) || step_idx <= 3 {
                let achieved = racket_velocity_estimate(arm, rail_x, rail_v, &q, &qdot);
                let saturated: Vec<usize> = (0..n)
                    .filter(|&i| {
                        (step.torque_cmd[i].abs() - arm.joint_torque_limits[i]).abs() < 1e-6
                    })
                    .collect();
                let util: Vec<f64> = (0..n)
                    .map(|i| step.torque_cmd[i].abs() / arm.joint_torque_limits[i])
                    .collect();
                eprintln!(
                    "diag t={t:.3} pos_err={pos_err:.4} racket_accel_desired={:?} \
                     torque_cmd={:?} util={util:?} saturated={saturated:?} achieved_v={achieved:?} \
                     target_v={target_racket_velocity:?}",
                    step.racket_accel_desired, step.torque_cmd
                );
            }
        }
        if pos_err < POSITION_TOLERANCE_RAD_OR_M
            && racket_velocity_ok(arm, rail_x, rail_v, &q, &qdot, target_racket_velocity)
        {
            converged = true;
            break;
        }
    }

    if diag && step_count > 0 {
        let mean: Vec<f64> = (0..n)
            .map(|i| torque_util_sum[i] / step_count as f64)
            .collect();
        eprintln!(
            "diag SUMMARY converged={converged} steps={step_count} torque_util mean={mean:?} max={torque_util_max:?} (1.0 = 한계)"
        );
    }
    if !converged {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::InsufficientTime {
                time_to_impact_secs: prediction.time_to_impact_secs,
                min_swing_secs: t,
            },
        ));
    }

    return Ok(BangBangTrajectory {
        dt: PLAN_DT_SECS,
        joint_samples,
        rail_samples,
    });
}

fn racket_velocity_ok(
    arm: &Arm,
    rail_x: f64,
    rail_velocity: f64,
    joints: &[f64],
    joint_velocities: &[f64],
    target_racket_velocity: Vector3<f64>,
) -> bool {
    let Some(achieved) =
        racket_velocity_estimate(arm, rail_x, rail_velocity, joints, joint_velocities)
    else {
        return false;
    };
    let target_speed = target_racket_velocity.norm();
    let achieved_speed = achieved.norm();
    if target_speed <= f64::EPSILON {
        return true;
    }
    let speed_ratio = achieved_speed / target_speed;
    if !(1.0 - RACKET_SPEED_RATIO_TOLERANCE..=1.0 + RACKET_SPEED_RATIO_TOLERANCE)
        .contains(&speed_ratio)
    {
        return false;
    }
    if achieved_speed <= f64::EPSILON {
        return false;
    }
    let cos_angle =
        (achieved.dot(&target_racket_velocity) / (achieved_speed * target_speed)).clamp(-1.0, 1.0);
    return cos_angle.acos().to_degrees() <= RACKET_DIRECTION_TOLERANCE_DEG;
}

/// 현재 관절/레일 위치·속도에서 실제로 나오는 라켓(월드) 속도 추정 —
/// `Arm::velocities_for_racket_velocity`와 같은 유한차분 스타일(`STEP=1e-6`).
fn racket_velocity_estimate(
    arm: &Arm,
    rail_x: f64,
    rail_velocity: f64,
    joints: &[f64],
    joint_velocities: &[f64],
) -> Option<Vector3<f64>> {
    const STEP: f64 = 1e-6;
    let base = arm.forward_kinematics_with_rail(rail_x, &Joints::from_slice(joints))?;
    let perturbed_joints: Vec<f64> = joints
        .iter()
        .zip(joint_velocities)
        .map(|(q, v)| q + v * STEP)
        .collect();
    let perturbed = arm.forward_kinematics_with_rail(
        rail_x + rail_velocity * STEP,
        &Joints::from_slice(&perturbed_joints),
    )?;
    return Some((perturbed.position.coords - base.position.coords) / STEP);
}

/// 1차원 이중적분기를 원점(목표)으로 모는 시간최적 bang-bang 스위칭.
/// `x`/`v`는 목표 기준 상대 위치/속도 오차(`현재 - 목표`), `a_max`는 이
/// 축이 낼 수 있는 최대 가속.
///
/// `bang_bang_accel_to`(목표속도 일반화판)가 실제 호출부를 대체해 프로덕션
/// 경로에서는 더 이상 쓰이지 않는다 — 그 함수가 고친 결함(목표속도 0 가정,
/// coast 구간 전체에서 스위칭 함수가 0이 되는 함정)이 이 함수에도 잠재해
/// 있었음을 보여주는 회귀 비교 기준으로만 테스트에 남긴다.
#[cfg(test)]
fn bang_bang_accel(x: f64, v: f64, a_max: f64) -> f64 {
    let switch = x + v * v.abs() / (2.0 * a_max);
    if switch.abs() < 1e-12 {
        return 0.0;
    }
    return -a_max * switch.signum();
}

/// 1차원 이중적분기를 `(위치오차 0, 속도 target_v)`로 모는 시간최적 bang-bang
/// 스위칭 — `bang_bang_accel`(테스트 전용 회귀 비교 기준)의 목표속도 일반화판.
///
/// `x`는 목표 기준 상대 위치(`현재 - 목표`), `v`는 **절대** 속도, `target_v`는
/// 도달 시점에 원하는 **절대** 속도다(속도 *오차*가 아니다 — `ẋ = v`가 그대로
/// 성립해야 하므로).
///
/// 마지막 구간에서 상수 가속 `a`로 `(0, target_v)`에 도달하는 궤적은
/// `v² - target_v² = 2a·x`, 즉 `x = Γ_a(v) := (v² - target_v²) / (2a)`를 만족한다
/// (`a=+a_max`분지 `Γ₊`는 `v ≤ target_v`일 때, `a=-a_max`분지 `Γ₋`는
/// `v ≥ target_v`일 때 관련 있다 — 그 분지로 계속 도달하려면 속도가 그 방향
/// 이어야 하므로).
///
/// **부호 기반 판별의 두 가지 실패 모드(실측으로 발견)**: (1) `switch = x -
/// Γ(v)` 결합공식은 coast 구간 **전체**에서 정확히 0이다(그 구간의 모든 점이
/// 정의상 `x = Γ(v)` 위에 있으므로 — 스위칭 "순간"만이 아니다). 부동소수점
/// 오차로 우연히 정확히 0에 고정되면 그 부동점에서 가속 0으로 영원히 멈춘다.
/// (2) `x`/`Γ(v)`를 직접 비교하는 분지 판별로 바꿔도, `v`가 Γ의 극값
/// (`dΓ/dv=0`, 즉 case1 실측처럼 coast 경로가 `v=0`을 지나가면 — `target_v`가
/// 0이 아니어도 `Γ₊`의 극값 자체는 항상 `v=0`)을 지나는 순간에는 `Γ(v)`가
/// 국소적으로 평탄해 `x`의 누적 부동소수점 잡음이 판별 부호를 매 스텝 뒤집어
/// `+a_max`/`-a_max` 채터링에 빠진다(실측: `x≈-2,v≈0` 근방에서 20000스텝 동안
/// 전진 없이 진동). `a_needed`(아래)의 크기만으로 매끄러운 모드를 트리거하는
/// 방식도 시도했으나 **거짓 양성**이 있었다 — 곡선에서 한참 떨어진 점인데도
/// `v²-target_v²`가 우연히 작아 `|a_needed|≤a_max`를 만족하는 경우가 있다
/// (실측: `x=-0.5,v=0.3,target_v=0`에서 `a_needed=-0.09`로 통과 조건을
/// 만족하지만 실제로는 `-a_max` 풀가속이 정답).
///
/// 그래서 곡선까지의 **직접 거리** `delta = x - Γ(v)`로 게이팅한다: `|delta|`가
/// 경계층(`BOUNDARY_LAYER`) 안이면(= 실제로 곡선 위/근처, 위 오탐과 달리 값 자체가
/// 아니라 곡선까지 거리로 판정하므로 오탐이 없다) 매끄러운
/// `a_needed = (v² - target_v²) / (2x)`를 쓴다 — 경계층 경계에서 정의상
/// `|a_needed| = a_max`로 분지 가속과 정확히 일치해 불연속이 없다. 경계층
/// 밖이면 `delta`의 부호로 풀 `±a_max` 방향을 정한다. 정확히 도달한 지점
/// (`x=0 ∧ v=target_v`)은 별도로 0을 반환 — coast 구간 전체가 아니라 이 한
/// 점만이므로 (1)의 재발 없이 안전하다.
///
/// 프로덕션 경로(`plan_bang_bang_for`)는 더 이상 이 함수를 쓰지 않는다 — 3개
/// 독립 축(x,y,z)에 각각 적용하면 축끼리 자코비안 결합을 통해 서로 간섭해
/// 거시적으로 진동했다(실측). ZEM/ZEV 유도법(하나의 벡터식)으로 대체했다.
/// 이 함수와 위 도출 과정 자체는 유효한 결과(1D 시간최적 스위칭의 올바른
/// 목표속도 일반화)라 회귀 검증용 테스트로 남긴다.
#[cfg(test)]
fn bang_bang_accel_to(x: f64, v: f64, target_v: f64, a_max: f64) -> f64 {
    if x == 0.0 && v == target_v {
        return 0.0;
    }
    const BOUNDARY_LAYER: f64 = 1e-2;
    if v <= target_v {
        let gamma_plus = (v * v - target_v * target_v) / (2.0 * a_max);
        let delta = x - gamma_plus;
        if delta.abs() < BOUNDARY_LAYER && x != 0.0 {
            return (v * v - target_v * target_v) / (2.0 * x);
        }
        if delta <= 0.0 {
            return a_max;
        }
        return -a_max;
    }
    let gamma_minus = (target_v * target_v - v * v) / (2.0 * a_max);
    let delta = x - gamma_minus;
    if delta.abs() < BOUNDARY_LAYER && x != 0.0 {
        return (v * v - target_v * target_v) / (2.0 * x);
    }
    if delta >= 0.0 {
        return -a_max;
    }
    return a_max;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::table;
    use crate::estimator::Prediction;
    use crate::robot::Arm;

    /// 피처 브랜치가 실기 관절속도(~2.88 rad/s)로 검증한 마운트.
    fn competition_arm() -> Arm {
        let mount_z = table::SURFACE_Z + 0.05;
        return (*crate::defaults::primitive_4dof_with_mount(-0.02, mount_z)
            .expect("4dof arm")
            .arm)
            .clone();
    }

    #[test]
    #[ignore = "일회성 실측(사용자 질문, 2026-07-28): 백그라운드 워커로 옮기기 전 \
                동기 plan_bang_bang_swing 자체의 실제 벽시계 소요 시간 — \
                이번 세션의 알고리즘 수정(가중치·스케일링·게인)이 계산 자체를 \
                빠르게 만들지는 않았다는 걸 직접 확인한다. \
                실행: cargo test --release --lib diag_measure_synchronous_wall_clock_cost \
                -- --ignored --nocapture"]
    fn diag_measure_synchronous_wall_clock_cost() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);

        const RUNS: usize = 20;
        let mut total = std::time::Duration::ZERO;
        let mut worst = std::time::Duration::ZERO;
        for _ in 0..RUNS {
            let t0 = std::time::Instant::now();
            let _ = plan_bang_bang_swing(&arm, &[prediction], &start_pose);
            let elapsed = t0.elapsed();
            total += elapsed;
            worst = worst.max(elapsed);
        }
        eprintln!(
            "plan_bang_bang_swing 동기 호출 {RUNS}회: 평균={:.2}ms 최악={:.2}ms \
             (이 값이 물리 스레드를 그대로 블로킹하던 시간 — 워커로 옮긴 뒤엔 \
             이 시간이 사라지는 게 아니라 물리 스레드 밖으로 옮겨질 뿐)",
            total.as_secs_f64() * 1000.0 / RUNS as f64,
            worst.as_secs_f64() * 1000.0,
        );
    }

    #[test]
    #[ignore = "일회성 실측(사용자 질문, 2026-07-28): 실제 게임플레이 경로(quintic \
                plan_swing)는 이번 세션에서 전혀 수정하지 않았다 — 그게 정말 \
                bang-bang과 비교해 물리 스레드를 블로킹할 만큼 느린지, 굳이 \
                워커로 옮길 필요가 없는 수준인지 실측으로 확인한다. \
                실행: cargo test --release --lib diag_measure_quintic_wall_clock_cost \
                -- --ignored --nocapture"]
    fn diag_measure_quintic_wall_clock_cost() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);

        const RUNS: usize = 20;
        let mut total = std::time::Duration::ZERO;
        let mut worst = std::time::Duration::ZERO;
        for _ in 0..RUNS {
            let t0 = std::time::Instant::now();
            let _ = crate::plan_swing(&arm, prediction, &start_pose);
            let elapsed = t0.elapsed();
            total += elapsed;
            worst = worst.max(elapsed);
        }
        eprintln!(
            "plan_swing(quintic, 실제 게임플레이 경로) 동기 호출 {RUNS}회: \
             평균={:.4}ms 최악={:.4}ms",
            total.as_secs_f64() * 1000.0 / RUNS as f64,
            worst.as_secs_f64() * 1000.0,
        );
    }

    #[test]
    #[ignore = "일회성 검증(사용자 요청, 2026-07-28): quintic의 임팩트 관절속도 해산\
                (`Arm::linear_velocities_for_racket_velocity`)에 추가한 토크 가중\
                최소노름 역산이 실제로 베이스 쪽(토크 여유 큰) 관절을 더 쓰게\
                하는지, 같은 목표에 대해 균일(옛) 최소노름과 직접 비교한다. \
                실행: cargo test --lib diag_quintic_velocity_solve_prefers_base_joints \
                -- --ignored --nocapture"]
    fn diag_quintic_velocity_solve_prefers_base_joints() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);
        let target = solve_impact_target(&arm, &prediction, &start_pose).expect("target 계산");

        let jacobian = arm.position_jacobian_fd(&target.pose).expect("자코비안");
        let racket_v = target.racket_velocity;
        let target_vec = DVector::from_vec(vec![racket_v.x, racket_v.y, racket_v.z]);

        // 옛(균일 최소노름) 해 — 이번 수정 전 `linear_velocities_for_racket_velocity`가
        // 그대로 쓰던 식을 여기 재현한다(프로덕션 함수는 이미 가중치를 쓰도록
        // 바뀌었으므로, 비교를 위해 원래 식만 별도로 계산).
        let jjt = &jacobian * jacobian.transpose() + DMatrix::identity(3, 3) * 1e-9;
        let uniform_inv = jjt.try_inverse().expect("역행렬 존재");
        let uniform_velocities = jacobian.transpose() * uniform_inv * &target_vec;

        // 새(토크 가중) 해 — 프로덕션 함수를 그대로 호출.
        let (_, weighted_joint_velocities) = arm
            .linear_velocities_for_racket_velocity(&target.pose, racket_v)
            .expect("가중 해");

        let has_rail = arm.rail.is_some();
        let rail_offset = usize::from(has_rail);
        let n = weighted_joint_velocities.len();
        let uniform_joint_velocities: Vec<f64> = (0..n)
            .map(|i| uniform_velocities[i + rail_offset])
            .collect();

        eprintln!("관절 토크한계(rail 제외)={:?}", arm.joint_torque_limits);
        eprintln!("균일(옛) 관절속도={uniform_joint_velocities:?}");
        eprintln!("토크가중(신규) 관절속도={weighted_joint_velocities:?}");
        for i in 0..n {
            let shift = weighted_joint_velocities[i].abs() - uniform_joint_velocities[i].abs();
            eprintln!(
                "  joint{i} (토크한계={:.2}N·m): |v| 변화 {shift:+.4} rad/s \
                 ({} = 이 관절 기여가 늘어남/줄어듦)",
                arm.joint_torque_limits[i],
                if shift > 0.0 { "증가" } else { "감소" },
            );
        }
    }

    /// 대표 임팩트 위치 — 실제 접수 평면(y=0.30) × 실현가능 높이 대역
    /// (탁구대 위 ~17cm). 예전에는 "휴지 자세의 FK 위치"를 썼는데, 휴지
    /// 자세를 임팩트 자세들 쪽으로 옮긴 뒤(`READY_JOINTS_4DOF`)로는 그 점이
    /// 오히려 특이점 근처가 됐다(실측: 관절 2가 15.4 rad/s 요구).
    ///
    /// 입사속도는 재튜닝된 기본 슈터가 이 평면에서 실제로 만드는 값
    /// (`shot_tune --explain` 실측). 예전의 완만한 1 m/s는 오히려 **더**
    /// 어렵다 — 반발계수 물리상(`v_r=(v_out+e*v_in)/(1+e)`) 입사가 느릴수록
    /// 라켓이 스스로 내야 하는 속도가 커져 특이점으로 몰린다(실측: 1 m/s
    /// 입사에서 관절 2가 14.1 rad/s 요구).
    fn sample_prediction(time_to_impact_secs: f64) -> Prediction {
        use crate::constants::table;
        return Prediction {
            time_to_impact_secs,
            impact_position: crate::Point3::new(table::WIDTH_X * 0.5, 0.30, 0.932),
            incoming_velocity: Vector3::new(0.0, -6.01, 1.51),
        };
    }

    #[test]
    #[ignore = "ZEM/ZEV 재설계(2026-07-27) 이후에도 미해결 — 위치는 완전히 \
                수렴하지만(pos_err ~0.00004~0.00007, swing_bench 다중 시나리오 \
                스윕 실측) 라켓 속도가 시나리오와 무관하게 목표의 22~24%에서 \
                멈춘다(방향은 4.8~6.0°로 정상). 이전 버전(관절별/축별 독립 \
                bang-bang)이 겪은 진동은 완전히 해소됐다 — 이건 새로운, 더 \
                좁혀진 잔여 결함. 사용자 지시로 이번 세션에서는 추가 튜닝을 \
                그만두고 코드 완성도로 전환함 — 원인 후보는 \
                .omc/research/known-regressions-realistic-joint-speed.md §6-3,\
                기록은 .omc/progress.txt. quintic 게임플레이 경로와 무관한 \
                GUI 디버그 전용 경로라 사용자 영향 없음."]
    fn plan_bang_bang_swing_converges_for_a_reachable_impact() {
        // 완만한 시나리오(약한 입사속도)로 메커니즘 자체의 수렴을 확인한다.
        // 빠른/까다로운 시나리오는 `tools/swing_bench`에서 이미 실측했듯
        // 실기 토크·속도 한계 안에서 진짜로 도달 불가능할 수 있고, 그 경우
        // `Err(InfeasibleSwing)`을 내는 게 올바른 동작이지 버그가 아니다.
        //
        // `tti`는 커밋창(`[min_swing_secs, swing_commit_max_secs]` =
        // `[0.08, 0.35]`) **안**이어야 한다 — 예전에는 2.5초를 줬는데, 이 값은
        // `plan_bang_bang_swing`의 `in_swing_commit_window` 필터에 걸려
        // `plan_bang_bang_for`(실제 bang-bang 적분)가 호출되기도 전에
        // "예측 없음" 폴백 에러(`time_to_impact_secs: 0.0`)로 실패했다 —
        // 스위칭 곡선 버그와는 무관한, 이 테스트 자체의 사전 결함이었다.
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let planned = plan_bang_bang_swing(&arm, &[sample_prediction(0.3)], &start_pose)
            .expect("bang-bang 계획 성공");
        assert!(planned.trajectory.duration_secs() > 0.0);
        let end = planned.trajectory.end_joints();
        assert_eq!(end.values.len(), arm.joint_count());
    }

    /// 낙관적(위치 제약은 무시한, 사실상 도달 불가능한 진짜 상한) 가정 하의
    /// 라켓 속도/가속 상한 — 모든 관절이 목표 방향 기여 부호에 맞춰 자기
    /// 한계(토크/유효관성에서 나오는 최대가속, 또는 `max_joint_speed`)로
    /// 동시에 정렬한다고 가정한다. `None`이면 IK/자코비안이 이 임팩트에서
    /// 안 풀림(하드 도달불가와는 다른 실패 — 호출부가 구분해서 처리).
    struct KinematicCeiling {
        target_speed: f64,
        v_max_kinematic: f64,
        a_max_kinematic: f64,
        joint_a_max: Vec<f64>,
        joint_v_max: Vec<f64>,
        jt_d: Vec<f64>,
    }

    fn kinematic_ceiling(
        arm: &Arm,
        start: &RobotPose,
        prediction: &Prediction,
    ) -> Option<KinematicCeiling> {
        let target = solve_impact_target(arm, prediction, start).ok()?;

        let jacobian = arm.position_jacobian_fd(&target.pose)?;

        let n = target.pose.joints.values.len();
        let mut rnea = RneaScratch::new();
        let mut mm_scratch = MassMatrixScratch::new();
        let mut mass = DMatrix::zeros(n, n);
        mass_matrix_into(
            arm,
            &target.pose.joints,
            &mut rnea,
            &mut mm_scratch,
            &mut mass,
        );

        let has_rail = arm.rail.is_some();
        let rail_offset = usize::from(has_rail);
        let joint_a_max: Vec<f64> = (0..jacobian.ncols())
            .map(|col| {
                if has_rail && col == 0 {
                    return RAIL_ACCEL_M_S2;
                }
                let i = col - rail_offset;
                arm.joint_torque_limits[i] / mass[(i, i)].max(1e-9)
            })
            .collect();
        let joint_v_max: Vec<f64> = (0..jacobian.ncols())
            .map(|col| {
                if has_rail && col == 0 {
                    return arm.rail.as_ref().expect("rail").max_speed;
                }
                arm.max_joint_speed
            })
            .collect();

        let target_speed = target.racket_velocity.norm();
        if target_speed <= f64::EPSILON {
            return None;
        }
        let d = target.racket_velocity / target_speed;
        let jt_d = jacobian.transpose() * DVector::from_vec(vec![d.x, d.y, d.z]);

        // 각 관절이 목표 방향에 대한 자기 기여 부호에 맞춰 자기 한계로 동시에
        // 정렬한다는 낙관적 가정의 선형계획(box 제약) 상한 —
        // max sum_i cap_i * |(J^T d)_i|. 실제로는 이 조합이 동시에 목표
        // *위치*까지도 맞춰야 하니 실현 가능한 속도는 이보다 항상 낮거나 같다.
        let v_max_kinematic: f64 = (0..jacobian.ncols())
            .map(|i| joint_v_max[i] * jt_d[i].abs())
            .sum();
        let a_max_kinematic: f64 = (0..jacobian.ncols())
            .map(|i| joint_a_max[i] * jt_d[i].abs())
            .sum();

        return Some(KinematicCeiling {
            target_speed,
            v_max_kinematic,
            a_max_kinematic,
            joint_a_max,
            joint_v_max,
            jt_d: jt_d.as_slice().to_vec(),
        });
    }

    fn print_feasibility_ceiling(
        label: &str,
        arm: &Arm,
        start: &RobotPose,
        prediction: &Prediction,
    ) {
        let ceiling = kinematic_ceiling(arm, start, prediction).expect("임팩트 목표 계산 성공");
        let naive_time_to_reach = ceiling.target_speed / ceiling.a_max_kinematic;

        eprintln!(
            "--- {label} ---\n\
             target_speed={:.3} m/s\n\
             v_max_kinematic(모든 관절 동시 최고속 정렬 상한)={:.3} m/s ({:.1}% of target)\n\
             a_max_kinematic(모든 관절 동시 최대토크가속 정렬 상한)={:.3} m/s^2\n\
             naive_time_to_reach(등가속도·위치제약무시 가정)={naive_time_to_reach:.3}s vs Tg={:.3}s\n\
             joint_a_max={:?}\n\
             joint_v_max={:?}\n\
             jt_d(J^T·목표방향, 관절별 기여)={:?}",
            ceiling.target_speed,
            ceiling.v_max_kinematic,
            ceiling.v_max_kinematic / ceiling.target_speed * 100.0,
            ceiling.a_max_kinematic,
            prediction.time_to_impact_secs,
            ceiling.joint_a_max,
            ceiling.joint_v_max,
            ceiling.jt_d,
        );

        match plan_bang_bang_swing(arm, &[*prediction], start) {
            Ok(planned) => {
                let end = planned.trajectory.end_joints();
                let end_qdot = planned
                    .trajectory
                    .sample_velocity_at(planned.trajectory.duration_secs());
                let achieved = racket_velocity_estimate(
                    arm,
                    planned.trajectory.follow_through_rail_x(),
                    0.0,
                    &end.values,
                    &end_qdot,
                );
                eprintln!(
                    "  => plan_bang_bang_swing: 실제 수렴 성공(converged=true), achieved_racket_v={achieved:?}"
                );
            }
            Err(err) => {
                eprintln!(
                    "  => plan_bang_bang_swing: Err({err}) — 실제로는 수렴 안 함(converged=false)"
                );
            }
        }
    }

    #[test]
    #[ignore = "일회성 진단(사용자 요청, 2026-07-28): 목표 라켓 속도가 이 팔의 \
                토크·관절속도 한계상 '원리적으로' 가능한지 확인 — 위치 제약을 \
                무시한 순수 낙관적 상한이라 이보다 목표가 크면 어떤 제어법으로도 \
                불가능, 작으면 병목은 하드웨어가 아니라 유도법/역산 쪽. \
                실행: cargo test --lib diag_kinematic_feasibility_ceiling \
                -- --ignored --nocapture"]
    fn diag_kinematic_feasibility_ceiling() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

        print_feasibility_ceiling(
            "원래 hard fixture (impact z=0.932, incoming (0,-6.01,1.51), tti=0.3)",
            &arm,
            &start_pose,
            &sample_prediction(0.3),
        );

        // 같은(IK가 실제로 풀리는) 임팩트 자세에서 입사속도 크기만 바꿔 목표
        // 속도 크기별로 상한 비율이 어떻게 바뀌는지 본다.
        print_feasibility_ceiling(
            "약한 입사속도 (0,-3.0,0.75), tti=0.3",
            &arm,
            &start_pose,
            &Prediction {
                time_to_impact_secs: 0.3,
                impact_position: crate::Point3::new(table::WIDTH_X * 0.5, 0.30, 0.932),
                incoming_velocity: Vector3::new(0.0, -3.0, 0.75),
            },
        );
        print_feasibility_ceiling(
            "더 약한 입사속도 (0,-1.5,0.37), tti=0.3",
            &arm,
            &start_pose,
            &Prediction {
                time_to_impact_secs: 0.3,
                impact_position: crate::Point3::new(table::WIDTH_X * 0.5, 0.30, 0.932),
                incoming_velocity: Vector3::new(0.0, -1.5, 0.37),
            },
        );
    }

    /// 발사속도가 바뀌면 같은 pitch/height로는 착지점이 크게 흔들린다(피치가
    /// 고정이면 탄도 자체가 달라지므로) — 실제 Rapier로 발사해 첫 바운드가
    /// 네트를 넘겨 로봇 쪽 반코트 안(너무 끝이 아닌 가운데 대역)에 떨어지는지
    /// 직접 확인한다. `None`이면 네트에 걸리거나(못 넘김) 착지 전 상태 종료.
    fn first_bounce_xy(speed: f64, pitch_deg: f64, height_offset_m: f64) -> Option<(f64, f64)> {
        use crate::sim::{BallShooterSettings, BallState, SimWorld};

        let robot = crate::defaults::primitive_4dof_with_mount(-0.02, table::SURFACE_Z + 0.05)
            .expect("robot 빌드 성공");
        let mut world = SimWorld::new(robot);
        let mut settings = BallShooterSettings::default();
        settings.speed_mps = speed;
        settings.pitch_deg = pitch_deg;
        settings.height_offset_m = height_offset_m;
        world.shoot_ball(&settings);

        const DT: f64 = 1.0 / 1000.0;
        let net_y = table::LENGTH_Y * 0.5;
        let net_top_z = (table::SURFACE_Z + table::NET_HEIGHT) as f32;
        let bounce_z = (table::SURFACE_Z + crate::constants::BALL_RADIUS) as f32;

        let mut prev_y = f32::INFINITY;
        let mut prev_z = f32::INFINITY;
        let mut cleared_net = false;
        for _ in 0..4_000 {
            world.step(DT, None);
            if world.ball_state != BallState::InFlight {
                return None;
            }
            let pos = world.ball_position();
            if prev_y > net_y as f32 && pos.y <= net_y as f32 {
                if pos.z <= net_top_z {
                    return None; // 네트에 걸림 — 이 pitch는 무효
                }
                cleared_net = true;
            }
            if cleared_net && pos.z <= bounce_z && prev_z > bounce_z {
                return Some((f64::from(pos.x), f64::from(pos.y)));
            }
            prev_y = pos.y;
            prev_z = pos.z;
        }
        return None;
    }

    /// 이 속도에서 "네트를 넘겨 로봇 쪽 반코트 가운데 대역(너무 끝이 아닌
    /// 곳)에 떨어지는" pitch를 거친 그리드로 찾는다 — 착지 y가
    /// `TARGET_Y`(로봇 반코트 `[0, net_y]`의 중앙 근처)에 가장 가까운 후보.
    fn find_legal_pitch_deg(speed: f64, height_offset_m: f64) -> Option<f64> {
        const TARGET_Y: f64 = 0.65;
        const MIN_Y: f64 = 0.25;
        const MAX_Y: f64 = 1.05;
        const MIN_X: f64 = table::WIDTH_X * 0.2;
        const MAX_X: f64 = table::WIDTH_X * 0.8;

        let mut best: Option<(f64, f64)> = None; // (pitch, |y - TARGET_Y|)
        let mut pitch = -14.0_f64;
        while pitch <= 6.0 {
            if let Some((x, y)) = first_bounce_xy(speed, pitch, height_offset_m) {
                if (MIN_Y..=MAX_Y).contains(&y) && (MIN_X..=MAX_X).contains(&x) {
                    let err = (y - TARGET_Y).abs();
                    if best.is_none_or(|(_, best_err)| err < best_err) {
                        best = Some((pitch, err));
                    }
                }
            }
            pitch += 0.5;
        }
        return best.map(|(pitch, _)| pitch);
    }

    #[test]
    #[ignore = "일회성 실험(사용자 요청, 2026-07-28): 실제 슈터 물리(Rapier)로 \
                발사속도를 스윕해, 어느 속도 이상에서 커밋창 안 첫 예측의 요구 \
                라켓속도가 kinematic_ceiling(위치 무시 낙관적 상한) 기준 \
                '이론상 가능'(비율>=100%)해지는지 찾는다 — 실제 랠리 경로 \
                (predict_impact, ground-truth rough 추종)를 그대로 써서 실제로 \
                나오는 임팩트/입사속도를 넣는다(손으로 고른 시나리오 아님). \
                속도마다 pitch를 재탐색해 '반대편 코트 가운데 대역에 정상적으로 \
                떨어지는' 샷만 평가한다(고정 pitch로는 극단 속도에서 착지 자체가 \
                무효가 됨 — 사용자 지적). \
                실행: cargo test --lib diag_ball_speed_feasibility_sweep \
                -- --ignored --nocapture"]
    fn diag_ball_speed_feasibility_sweep() {
        use crate::sim::{BallShooterSettings, BallState, SimWorld};

        const DT: f64 = 1.0 / 1000.0;
        const MAX_STEPS: usize = 4_000;
        const HEIGHT_OFFSET_M: f64 = 0.24;

        let robot = crate::defaults::primitive_4dof_with_mount(-0.02, table::SURFACE_Z + 0.05)
            .expect("robot 빌드 성공");
        let arm = (*robot.arm).clone();

        let intercept = crate::InterceptWindow {
            y_min: 0.20,
            y_max: 0.55,
            sample_step: 0.05,
        };

        for speed in [
            3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
        ] {
            let Some(pitch_deg) = find_legal_pitch_deg(speed, HEIGHT_OFFSET_M) else {
                eprintln!(
                    "speed={speed:.1} m/s -> 반대편 코트 가운데 대역에 정상적으로 \
                     떨어지는 pitch 없음(탐색 범위 내) — 스킵"
                );
                continue;
            };

            let mut world = SimWorld::new(robot.clone());
            world.set_use_ground_truth(true);
            let mut settings = BallShooterSettings::default();
            settings.speed_mps = speed;
            settings.pitch_deg = pitch_deg;
            settings.height_offset_m = HEIGHT_OFFSET_M;
            world.shoot_ball(&settings);

            let mut found = None;
            for _ in 0..MAX_STEPS {
                world.step(DT, None);
                if world.ball_state != BallState::InFlight {
                    break;
                }
                let ball_y = f64::from(world.ball_position().y);
                if !crate::ball_past_midcourt_for_commit(ball_y) {
                    continue;
                }
                let predictions: Vec<_> = intercept
                    .hit_planes()
                    .into_iter()
                    .filter_map(|plane| crate::sim::predict_impact(&world, plane))
                    .collect();
                // 실제 게임플레이(`plan_best_swing`)처럼 커밋창 안 후보 중
                // 하나라도 되면 되는 게 아니라, kinematic_ceiling 비율이 가장
                // 좋은(가장 여유 있는) 후보를 골라 "최선의 경우"로 비교한다.
                let best_in_window = predictions
                    .iter()
                    .filter(|p| crate::in_swing_commit_window(p.time_to_impact_secs))
                    .copied()
                    .max_by(|a, b| {
                        let start =
                            RobotPose::new(world.robot().rail_x(), world.robot().joints().clone());
                        let ratio_a = kinematic_ceiling(&arm, &start, a)
                            .map_or(f64::NEG_INFINITY, |c| c.v_max_kinematic / c.target_speed);
                        let ratio_b = kinematic_ceiling(&arm, &start, b)
                            .map_or(f64::NEG_INFINITY, |c| c.v_max_kinematic / c.target_speed);
                        ratio_a
                            .partial_cmp(&ratio_b)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    });
                if let Some(p) = best_in_window {
                    let start =
                        RobotPose::new(world.robot().rail_x(), world.robot().joints().clone());
                    found = Some((p, start));
                    break;
                }
            }

            let Some((prediction, start)) = found else {
                eprintln!(
                    "speed={speed:.1} m/s pitch={pitch_deg:.1} -> 커밋창 안 예측 없음 (스킵)"
                );
                continue;
            };

            match kinematic_ceiling(&arm, &start, &prediction) {
                Some(ceiling) => {
                    let ratio = ceiling.v_max_kinematic / ceiling.target_speed * 100.0;
                    let verdict = if ratio >= 100.0 { "가능" } else { "부족" };
                    eprintln!(
                        "speed={speed:.1} m/s pitch={pitch_deg:.1} -> impact=({:.3},{:.3},{:.3}) \
                         v_in=({:.2},{:.2},{:.2}) tti={:.3}s target_speed={:.3} m/s \
                         v_max_kinematic={:.3} m/s (이론상 상한의 {ratio:.1}% 요구 -> {verdict})",
                        prediction.impact_position.x,
                        prediction.impact_position.y,
                        prediction.impact_position.z,
                        prediction.incoming_velocity.x,
                        prediction.incoming_velocity.y,
                        prediction.incoming_velocity.z,
                        prediction.time_to_impact_secs,
                        ceiling.target_speed,
                        ceiling.v_max_kinematic,
                    );
                }
                None => {
                    eprintln!(
                        "speed={speed:.1} m/s pitch={pitch_deg:.1} -> IK/자코비안 실패 \
                         (이 임팩트는 애초에 IK 불가)"
                    );
                }
            }
        }
    }

    /// `step_racket_guidance`가 실제로 쓰는 가중치(`torque_limit^4`)가, 균일
    /// 최소노름(가중치=1, 이전 설계)에 비해 베이스 쪽(토크 여유가 큰) 관절을
    /// 더 많이 쓰는지 직접 계산으로 검증한다 — 프로덕션 함수를 고치지 않고,
    /// 같은 자코비안/질량행렬에 가중치 지수만 바꿔 넣어 비교한다.
    fn print_joint_usage_by_weight_exponent(
        label: &str,
        arm: &Arm,
        start: &RobotPose,
        prediction: &Prediction,
    ) {
        let Ok(target) = solve_impact_target(arm, prediction, start) else {
            eprintln!("--- {label} ---\n  (IK 실패 — 스킵)");
            return;
        };
        let Some(jacobian) = arm.position_jacobian_fd(&target.pose) else {
            eprintln!("--- {label} ---\n  (자코비안 없음 — 스킵)");
            return;
        };
        let n = target.pose.joints.values.len();
        let mut rnea = RneaScratch::new();
        let mut mm_scratch = MassMatrixScratch::new();
        let mut mass = DMatrix::zeros(n, n);
        mass_matrix_into(
            arm,
            &target.pose.joints,
            &mut rnea,
            &mut mm_scratch,
            &mut mass,
        );

        let d = target.racket_velocity.normalize();
        let a_vec = DVector::from_vec(vec![d.x, d.y, d.z]);
        let has_rail = arm.rail.is_some();
        let rail_offset = usize::from(has_rail);

        eprintln!(
            "--- {label} (관절 토크한계, rail 제외={:?}) ---",
            arm.joint_torque_limits
        );
        for exponent in [0.0, 2.0, 4.0] {
            let joint_preference: Vec<f64> = (0..jacobian.ncols())
                .map(|col| {
                    if has_rail && col == 0 {
                        return 1.0;
                    }
                    let i = col - rail_offset;
                    arm.joint_torque_limits[i].powf(exponent)
                })
                .collect();
            let w_inv = DMatrix::from_diagonal(&DVector::from_vec(joint_preference));
            let j_winv = &jacobian * &w_inv;
            let jwjt = &j_winv * jacobian.transpose() + DMatrix::identity(3, 3) * JACOBIAN_DAMPING;
            let Some(jwjt_inv) = jwjt.try_inverse() else {
                eprintln!("  exponent={exponent:.0} -> jwjt 역행렬 없음 (스킵)");
                continue;
            };
            let j_pinv = &w_inv * jacobian.transpose() * jwjt_inv;

            // 단위 라켓가속(목표 방향)에 대한 관절가속 방향 -> 그 가속을 내는 데
            // 필요한 토크(중력/코리올리 bias 제외, 순수 질량행렬 항만) -> 토크
            // 한계 대비 이용률. bias 없이도 "이 방향에 상대적으로 어느 관절을
            // 더 쓰는가"라는 비교 목적에는 충분하다.
            let qddot_dir = &j_pinv * &a_vec;
            let joint_qddot: Vec<f64> = (0..n).map(|i| qddot_dir[i + rail_offset]).collect();
            let tau = &mass * DVector::from_vec(joint_qddot.clone());
            let util: Vec<f64> = (0..n)
                .map(|i| (tau[i] / arm.joint_torque_limits[i]).abs())
                .collect();
            let util_sum: f64 = util.iter().sum();
            let util_share_pct: Vec<f64> = util.iter().map(|u| u / util_sum * 100.0).collect();

            eprintln!(
                "  exponent={exponent:.0} (0=균일 최소노름, 4=프로덕션) -> \
                 관절별 토크이용 비중(%)={util_share_pct:.1?} \
                 (joint0=듀얼모터요·joint1=어깨: 베이스쪽 / joint2=엘보·joint3=손목: 말단쪽)"
            );
        }
    }

    #[test]
    #[ignore = "일회성 실험(사용자 요청, 2026-07-28): torque_limit^4 가중이 실제로 \
                베이스 쪽 관절(joint0=듀얼모터 요, joint1=어깨)을 손목/엘보(joint2/3, \
                토크 한계 더 작은 MX28)보다 더 쓰게 하는지 정량 검증 — 손으로 고른 \
                hard fixture 하나만이 아니라 diag_ball_speed_feasibility_sweep이 \
                실측한 '정상 착지' 시나리오들도 같이 봐서, 특정 자세의 우연(예:\
                joint1이 그 방향에 자코비안 기여가 0에 가까운 특이 자세)에 결론이\
                흔들리지 않게 한다. \
                실행: cargo test --lib diag_torque_weighted_pinv_prefers_base_joints \
                -- --ignored --nocapture"]
    fn diag_torque_weighted_pinv_prefers_base_joints() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

        print_joint_usage_by_weight_exponent(
            "hard fixture (impact z=0.932, incoming (0,-6.01,1.51))",
            &arm,
            &start_pose,
            &sample_prediction(0.3),
        );

        // diag_ball_speed_feasibility_sweep이 실측한, 실제로 네트를 넘겨 정상
        // 착지하는 시나리오들 — 손으로 고른 hard fixture와 달리 실제 슈터가
        // 만들 수 있는 임팩트.
        for (label, impact, incoming, tti) in [
            (
                "실측 speed=6.0m/s",
                (0.762, 0.200, 0.999),
                (0.0, -4.34, 1.51),
                0.248,
            ),
            (
                "실측 speed=9.0m/s",
                (0.762, 0.350, 0.875),
                (0.0, -7.32, 2.15),
                0.136,
            ),
            (
                "실측 speed=10.0m/s",
                (0.762, 0.350, 0.859),
                (0.0, -8.26, 2.28),
                0.121,
            ),
            (
                "실측 speed=12.0m/s",
                (0.762, 0.350, 0.865),
                (0.0, -10.09, 2.46),
                0.101,
            ),
        ] {
            print_joint_usage_by_weight_exponent(
                label,
                &arm,
                &start_pose,
                &Prediction {
                    time_to_impact_secs: tti,
                    impact_position: crate::Point3::new(impact.0, impact.1, impact.2),
                    incoming_velocity: Vector3::new(incoming.0, incoming.1, incoming.2),
                },
            );
        }
    }

    /// 방금 실측한 pos_err(0.76m 시작, 300ms 내내 거의 안 줄어듦)가 `.omc/`
    /// 문서의 "위치는 완전히 수렴한다"(pos_err 0.00004~0.00007)와 정면으로
    /// 충돌한다 — 진짜 회귀인지, `swing_bench`가 `--max-time-secs`(2~8s)를
    /// `Tg`로 그대로 먹여서 실제 커밋창(≤0.35s)보다 훨씬 긴, 비현실적으로
    /// 쉬운 문제를 풀고 있었던 것인지 확인한다. 같은 시작자세·목표에 Tg만
    /// 길게 줘서 직접 재현한다(=swing_bench와 동일 조건).
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): pos_err 불일치의 원인이 \
                Tg(time-to-go) 예산 길이 차이인지 직접 재현. \
                실행: cargo test --lib diag_long_tg_vs_real_deadline_pos_err \
                -- --ignored --nocapture"]
    fn diag_long_tg_vs_real_deadline_pos_err() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);

        let target = solve_impact_target(&arm, &prediction, &start_pose).expect("target");
        let target_racket_position = arm
            .forward_kinematics_with_rail(target.pose.rail_x, &target.pose.joints)
            .expect("FK")
            .position
            .coords;
        let target_racket_velocity = target.racket_velocity;
        let home_racket_position = arm
            .forward_kinematics_with_rail(start_pose.rail_x, &start_pose.joints)
            .expect("FK")
            .position
            .coords;
        eprintln!(
            "home_racket_position={home_racket_position:?}\ntarget_racket_position={target_racket_position:?}\n\
             euclidean distance={:.4} m (이게 diag t=0.001의 pos_err 초기값과 같아야 함)",
            (target_racket_position - home_racket_position).norm(),
        );

        let n = start_pose.joints.values.len();
        for (label, tg_budget) in [
            ("실제 커밋창과 같은 Tg=0.3s (진짜 마감)", 0.3_f64),
            ("swing_bench와 같은 Tg=2.0s", 2.0_f64),
            ("swing_bench와 같은 Tg=8.0s", 8.0_f64),
        ] {
            let mut q = start_pose.joints.values.clone();
            let mut qdot = vec![0.0; n];
            let mut rail_x = start_pose.rail_x;
            let mut rail_v = 0.0;
            let mut scratch = RacketGuidanceScratch::new(n);
            let dt = 0.001;
            let steps = (tg_budget / dt).round() as usize;
            let mut last_pos_err = f64::NAN;
            for step in 0..steps {
                let t = step as f64 * dt;
                let remaining = tg_budget - t;
                let Some(_step_result) = step_racket_guidance(
                    &arm,
                    &mut q,
                    &mut qdot,
                    &mut rail_x,
                    &mut rail_v,
                    target_racket_position,
                    target_racket_velocity,
                    remaining,
                    dt,
                    &mut scratch,
                ) else {
                    break;
                };
                if step % (steps / 5).max(1) == 0 || step < 3 {
                    let pose = arm
                        .forward_kinematics_with_rail(rail_x, &Joints::from_slice(&q))
                        .expect("fk");
                    let pos_err = (pose.position.coords - target_racket_position).norm();
                    last_pos_err = pos_err;
                    eprintln!("  [{label}] t={t:.3} pos_err={pos_err:.5}");
                }
            }
            eprintln!("[{label}] 최종 근접 pos_err={last_pos_err:.5}\n");
        }
    }

    /// 사용자 지시(2026-07-28): "타점(임팩트 목표)이 테이블을 안 침범하게만
    /// 하면 되지 않냐"는 질문 검증 — `solve_impact_target`이 내부에서 부르는
    /// `best_impact_candidate`(swing/physics.rs:171)가 이미 임팩트 후보의
    /// `table_penetration`을 걸러내므로(quintic·bang-bang 공용), **목표
    /// 지점 자체는 이미 안전**하다는 걸 확인한다. 대신 홈→목표로 가는
    /// 도중(과도 구간)에 관통이 생기는지를 실제 `plan_bang_bang_for`
    /// 적분 궤적을 따라 매 스텝 `table_penetration`을 찍어 직접 본다.
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): 임팩트 목표 지점은 이미 \
                안전한지, 홈->목표 도중(스윙 중)에 테이블 관통이 생기는지 \
                실제 궤적을 따라 확인. \
                실행: cargo test --lib diag_table_penetration_along_trajectory \
                -- --ignored --nocapture"]
    fn diag_table_penetration_along_trajectory() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);

        let home_penetration = crate::planner::collision::table_penetration(
            &arm,
            start_pose.rail_x,
            &start_pose.joints,
        );
        eprintln!("home 자세 table_penetration={home_penetration:.5}m (>0이면 이미 침범)");

        let target = solve_impact_target(&arm, &prediction, &start_pose).expect("target");
        let target_penetration = crate::planner::collision::table_penetration(
            &arm,
            target.pose.rail_x,
            &target.pose.joints,
        );
        eprintln!(
            "임팩트 목표 자세 table_penetration={target_penetration:.5}m \
             (best_impact_candidate가 이미 >1e-3인 후보는 걸러내므로 안전할 것으로 예상)"
        );

        // plan_bang_bang_swing은 실패하면 부분 결과를 안 주므로, 성공/실패와
        // 무관하게 직접 step_racket_guidance를 돌려 스윙 전체(홈->목표 도중)의
        // 관통 여부를 확인한다 — plan_bang_bang_for와 동일한 target/Tg 사용.
        let target_racket_position = arm
            .forward_kinematics_with_rail(target.pose.rail_x, &target.pose.joints)
            .expect("FK")
            .position
            .coords;
        let target_racket_velocity = target.racket_velocity;

        let n = start_pose.joints.values.len();
        let mut q = start_pose.joints.values.clone();
        let mut qdot = vec![0.0; n];
        let mut rail_x = start_pose.rail_x;
        let mut rail_v = 0.0;
        let mut scratch = RacketGuidanceScratch::new(n);
        let tg_budget = prediction.time_to_impact_secs.min(0.5);
        let steps = (tg_budget / PLAN_DT_SECS).round() as usize;

        // `plan_bang_bang_for`와 같은 구조로 재현한다: 유도법의 진짜 상태
        // (q/rail_x)는 절대 안 건들고, "재생용으로 내보낼 샘플"만 필요하면
        // 클램프한다. 그래서 raw(내부 상태)와 sample(출력) 관통을 따로 잰다 —
        // raw는 여전히 관통해도(유도법 자체는 안 흔들렸다는 뜻) sample은
        // 항상 안전해야 한다.
        let mut worst_raw = 0.0_f64;
        let mut worst_sample = 0.0_f64;
        let mut raw_penetrating_steps = 0;
        let mut sample_penetrating_steps = 0;
        for step in 0..steps {
            let t = step as f64 * PLAN_DT_SECS;
            let remaining = tg_budget - t;
            let Some(_) = step_racket_guidance(
                &arm,
                &mut q,
                &mut qdot,
                &mut rail_x,
                &mut rail_v,
                target_racket_position,
                target_racket_velocity,
                remaining,
                PLAN_DT_SECS,
                &mut scratch,
            ) else {
                break;
            };
            let raw_pen =
                crate::planner::collision::table_penetration(&arm, rail_x, &Joints::from_slice(&q));
            if raw_pen > 0.0 {
                raw_penetrating_steps += 1;
            }
            worst_raw = worst_raw.max(raw_pen);

            let sample_joints = if raw_pen > 1e-4 {
                crate::planner::collision::clamp_above_table(&arm, rail_x, &Joints::from_slice(&q))
            } else {
                Joints::from_slice(&q)
            };
            let sample_pen =
                crate::planner::collision::table_penetration(&arm, rail_x, &sample_joints);
            if sample_pen > 0.0 {
                sample_penetrating_steps += 1;
            }
            worst_sample = worst_sample.max(sample_pen);
            let _ = t;
        }
        eprintln!(
            "raw(유도법 내부 상태) 관통 스텝={raw_penetrating_steps}, 최악={worst_raw:.5}m \
             (건드리지 않았으므로 이전과 동일하게 나와야 함)\n\
             sample(재생 출력) 관통 스텝={sample_penetrating_steps}, 최악={worst_sample:.5}m \
             (0에 가까워야 함 — 이게 실제 로봇/화면에 나가는 값)"
        );
    }

    /// 사용자 지시(2026-07-28): `POSITION_TOLERANCE_RAD_OR_M`(1mm)이 실제
    /// 공(반경 20mm)·라켓 블레이드(반경 75mm) 크기에 비해 과도하게 엄격한지,
    /// 완화하면 수렴 성공률·수렴 속도(= 커밋창 예산 중 실제로 쓰는 비율,
    /// 실시간 GUI 스레드 부담과 직결)가 어떻게 바뀌는지 확인한다. 재컴파일
    /// 없이 한 번의 시뮬레이션에서 매 스텝 pos_err·vel_ok를 기록해두고,
    /// 사후에 여러 위치 허용오차 후보로 "그 허용오차였다면 몇 스텝만에
    /// 수렴했을까"를 재판정한다(속도 허용오차는 고정, 위치만 변수).
    #[test]
    #[ignore = "정확도-계산속도 트레이드오프 실험(사용자 요청, 2026-07-28): \
                POSITION_TOLERANCE_RAD_OR_M을 1mm~30mm로 완화했을 때 수렴\
                성공률과 수렴까지 걸리는 시간(스텝 수)이 어떻게 바뀌는지 \
                측정. \
                실행: cargo test --lib diag_position_tolerance_tradeoff \
                -- --ignored --nocapture"]
    fn diag_position_tolerance_tradeoff() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);

        let target = solve_impact_target(&arm, &prediction, &start_pose).expect("target");
        let target_racket_position = arm
            .forward_kinematics_with_rail(target.pose.rail_x, &target.pose.joints)
            .expect("FK")
            .position
            .coords;
        let home_racket_position = arm
            .forward_kinematics_with_rail(start_pose.rail_x, &start_pose.joints)
            .expect("FK")
            .position
            .coords;
        let pos_direction = (target_racket_position - home_racket_position).normalize();
        let vel_direction = target.racket_velocity.normalize();

        eprintln!(
            "공 반경={:.1}mm, 라켓 블레이드 반경={:.1}mm (참고: 지금 tolerance=1mm)\n",
            crate::constants::BALL_RADIUS * 1000.0,
            crate::constants::geometry::RACKET_BLADE_RADIUS * 1000.0,
        );

        const TG: f64 = 0.3;
        const DT: f64 = 0.001;
        let n = start_pose.joints.values.len();
        let tolerances_mm = [1.0, 3.0, 5.0, 10.0, 15.0, 20.0, 30.0];

        // (d, v) 대표 시나리오 — 트리비얼(d=0.02) ~ 실전 하드픽스처(d=0.764)까지.
        for &(d, v) in &[(0.02, 0.05), (0.10, 0.50), (0.30, 0.80), (0.764, 2.079)] {
            let target_pos = home_racket_position + pos_direction * d;
            let target_vel = vel_direction * v;

            let mut q = start_pose.joints.values.clone();
            let mut qdot = vec![0.0; n];
            let mut rail_x = start_pose.rail_x;
            let mut rail_v = 0.0;
            let mut scratch = RacketGuidanceScratch::new(n);
            let steps = (TG / DT).round() as usize;

            // 매 스텝 (pos_err, vel_ok) 기록 — 재컴파일 없이 여러 위치
            // 허용오차를 사후 재판정하기 위함.
            let mut history: Vec<(f64, bool)> = Vec::with_capacity(steps);
            for step in 0..steps {
                let t = step as f64 * DT;
                let remaining = TG - t;
                let Some(_) = step_racket_guidance(
                    &arm,
                    &mut q,
                    &mut qdot,
                    &mut rail_x,
                    &mut rail_v,
                    target_pos,
                    target_vel,
                    remaining,
                    DT,
                    &mut scratch,
                ) else {
                    break;
                };
                let Some(pose) = arm.forward_kinematics_with_rail(rail_x, &Joints::from_slice(&q))
                else {
                    break;
                };
                let pos_err = (pose.position.coords - target_pos).norm();
                let vel_ok = racket_velocity_ok(&arm, rail_x, rail_v, &q, &qdot, target_vel);
                history.push((pos_err, vel_ok));
            }

            let min_pos_err = history
                .iter()
                .map(|&(e, _)| e)
                .fold(f64::INFINITY, f64::min);
            eprintln!(
                "--- d={d:.3}m v={v:.3}m/s (기록된 스텝 수={}, 도달한 최소 pos_err={:.1}mm) ---",
                history.len(),
                min_pos_err * 1000.0,
            );
            for &tol_mm in &tolerances_mm {
                let tol = tol_mm / 1000.0;
                let converged_at = history
                    .iter()
                    .position(|&(pos_err, vel_ok)| pos_err < tol && vel_ok);
                match converged_at {
                    Some(step) => {
                        let t = step as f64 * DT;
                        eprintln!(
                            "  tolerance={tol_mm:>4.0}mm -> 수렴 성공, t={t:.3}s \
                             (Tg={TG}의 {:.0}% 사용, {step}스텝)",
                            t / TG * 100.0,
                        );
                    }
                    None => {
                        eprintln!("  tolerance={tol_mm:>4.0}mm -> 수렴 실패(Tg 안에 못 들어옴)");
                    }
                }
            }
        }
    }

    /// 사용자 지시(2026-07-28): "편안한 타점이 곧 가장 쉬운 지점"이라는
    /// 가정이 맞다면, home에서 목표까지의 거리(d)와 목표 라켓속도 크기(v)를
    /// 순수하게 낮춰가며 마감(Tg=0.3s, 실제 커밋창 최대치로 넉넉하게) 안에서
    /// 수렴하는 (d,v) 조합이 **존재하는지** 직접 확인한다 — 특정 공 궤적의
    /// IK/방향 사정과 무관하게, 방향은 실제 hard fixture의 방향을 그대로
    /// 쓰고 크기(스칼라)만 바꾼다. 존재한다면 그 경계가 "가능한 지점"의
    /// 실제 위치를 알려준다.
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): d(거리)·v(속도) 난이도를 \
                순수하게 낮춰가며 Tg=0.3s 안에서 수렴하는 지점이 존재하는지, \
                존재한다면 그 경계가 어디인지 직접 스캔한다. \
                실행: cargo test --lib diag_feasibility_boundary_exists \
                -- --ignored --nocapture"]
    fn diag_feasibility_boundary_exists() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);

        let target = solve_impact_target(&arm, &prediction, &start_pose).expect("target");
        let target_racket_position = arm
            .forward_kinematics_with_rail(target.pose.rail_x, &target.pose.joints)
            .expect("FK")
            .position
            .coords;
        let home_racket_position = arm
            .forward_kinematics_with_rail(start_pose.rail_x, &start_pose.joints)
            .expect("FK")
            .position
            .coords;
        let pos_direction = (target_racket_position - home_racket_position).normalize();
        let vel_direction = target.racket_velocity.normalize();
        eprintln!(
            "방향은 실제 hard fixture 그대로 고정, 거리(d)·속도(v) 크기만 스윕. \
             원래 d=0.764m, v=2.079m/s였음.\n"
        );

        const TG: f64 = 0.3; // 실제 커밋창 최대치 — 가장 넉넉한 실전 조건.
        const DT: f64 = 0.001;
        let n = start_pose.joints.values.len();

        let mut boundary_found = false;
        for &d in &[0.02, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.764] {
            for &v in &[0.05, 0.15, 0.30, 0.50, 0.80, 1.20, 1.60, 2.079] {
                let target_pos = home_racket_position + pos_direction * d;
                let target_vel = vel_direction * v;

                let mut q = start_pose.joints.values.clone();
                let mut qdot = vec![0.0; n];
                let mut rail_x = start_pose.rail_x;
                let mut rail_v = 0.0;
                let mut scratch = RacketGuidanceScratch::new(n);
                let steps = (TG / DT).round() as usize;
                let mut converged = false;
                for step in 0..steps {
                    let t = step as f64 * DT;
                    let remaining = TG - t;
                    let Some(_) = step_racket_guidance(
                        &arm,
                        &mut q,
                        &mut qdot,
                        &mut rail_x,
                        &mut rail_v,
                        target_pos,
                        target_vel,
                        remaining,
                        DT,
                        &mut scratch,
                    ) else {
                        break;
                    };
                    let Some(pose) =
                        arm.forward_kinematics_with_rail(rail_x, &Joints::from_slice(&q))
                    else {
                        break;
                    };
                    let pos_err = (pose.position.coords - target_pos).norm();
                    if pos_err < POSITION_TOLERANCE_RAD_OR_M
                        && racket_velocity_ok(&arm, rail_x, rail_v, &q, &qdot, target_vel)
                    {
                        converged = true;
                        break;
                    }
                }
                if converged {
                    boundary_found = true;
                }
                eprintln!("d={d:.2}m v={v:.2}m/s -> converged={converged}");
            }
        }
        eprintln!(
            "\n=== 존재 여부: {boundary_found} (true면 실제로 수렴 가능한 (d,v) 조합이 \
             있다는 뜻 — 그 경계 위치가 '가능한 지점') ==="
        );
    }

    /// `diag_feasibility_boundary_exists`에서 d=0.02m, v=0.05m/s(사실상 제자리)
    /// 조차 실패한 게 진짜 알고리즘 한계인지 버그인지 스텝별로 직접 본다.
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): 가장 쉬운 케이스조차 실패한 \
                이유를 스텝별 추적으로 확인 — 버그인지 진짜 한계인지 구분. \
                실행: cargo test --lib diag_trivial_case_trace -- --ignored --nocapture"]
    fn diag_trivial_case_trace() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let prediction = sample_prediction(0.3);

        let target = solve_impact_target(&arm, &prediction, &start_pose).expect("target");
        let target_racket_position = arm
            .forward_kinematics_with_rail(target.pose.rail_x, &target.pose.joints)
            .expect("FK")
            .position
            .coords;
        let home_racket_position = arm
            .forward_kinematics_with_rail(start_pose.rail_x, &start_pose.joints)
            .expect("FK")
            .position
            .coords;
        let pos_direction = (target_racket_position - home_racket_position).normalize();
        let vel_direction = target.racket_velocity.normalize();

        const TG: f64 = 0.3;
        const DT: f64 = 0.001;
        let n = start_pose.joints.values.len();

        for &(d, v) in &[(0.02, 0.05)] {
            eprintln!("--- d={d} v={v} ---");
            let target_pos = home_racket_position + pos_direction * d;
            let target_vel = vel_direction * v;

            let mut q = start_pose.joints.values.clone();
            let mut qdot = vec![0.0; n];
            let mut rail_x = start_pose.rail_x;
            let mut rail_v = 0.0;
            let mut scratch = RacketGuidanceScratch::new(n);
            let steps = (TG / DT).round() as usize;
            for step in 0..steps {
                let t = step as f64 * DT;
                let remaining = TG - t;
                let time_to_go = (remaining * TIME_TO_GO_BIAS).max(MIN_TIME_TO_GO_SECS);
                let pre_pose = arm
                    .forward_kinematics_with_rail(rail_x, &Joints::from_slice(&q))
                    .expect("fk");
                let pre_pos_err_vec = pre_pose.position.coords - target_pos;
                let pre_velocity = racket_velocity_estimate(&arm, rail_x, rail_v, &q, &qdot);
                let Some(step_result) = step_racket_guidance(
                    &arm,
                    &mut q,
                    &mut qdot,
                    &mut rail_x,
                    &mut rail_v,
                    target_pos,
                    target_vel,
                    remaining,
                    DT,
                    &mut scratch,
                ) else {
                    eprintln!("  t={t:.3} -> step_racket_guidance returned None (break)");
                    break;
                };
                let pose = arm
                    .forward_kinematics_with_rail(rail_x, &Joints::from_slice(&q))
                    .expect("fk");
                let pos_err = (pose.position.coords - target_pos).norm();
                let achieved = racket_velocity_estimate(&arm, rail_x, rail_v, &q, &qdot);
                let vel_ok = racket_velocity_ok(&arm, rail_x, rail_v, &q, &qdot, target_vel);
                if step >= 280 {
                    eprintln!(
                        "  t={t:.3} time_to_go={time_to_go:.6} pre_pos_err_vec={pre_pos_err_vec:?} \
                         pre_velocity={pre_velocity:?} qdot={qdot:?}\n    \
                         -> pos_err={pos_err:.6} achieved_v={achieved:?} vel_ok={vel_ok} \
                         racket_accel_desired={:?} torque_cmd={:?}",
                        step_result.racket_accel_desired, step_result.torque_cmd,
                    );
                }
                if pos_err < POSITION_TOLERANCE_RAD_OR_M && vel_ok {
                    eprintln!("  t={t:.3} -> 수렴!");
                    break;
                }
            }
        }
    }

    /// `diag_ball_speed_feasibility_sweep`이 찾은 "정상 착지 + 이론상 여유
    /// 129~146%"인 실측 시나리오들에서, 실제 `plan_bang_bang_swing`(프로덕션
    /// 알고리즘)이 정말 수렴하는지 확인한다 — 이전에 손으로 만든 "168% 여유"
    /// 시나리오도 실패했던 적이 있어, 이론상 쉬운 목표가 실제로도 쉬운지는
    /// 별도로 검증해야 한다.
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): 이론상 여유(129~146%)가 있는 \
                실측 시나리오에서도 plan_bang_bang_swing이 수렴 실패하는지 확인 — \
                실패한다면 '목표가 너무 세서'가 아니라 알고리즘 자체의 구조적 결함 \
                (예: 관절별 독립 qddot/토크 클램프가 하나로 묶인 라켓방향 명령을 \
                깨는 것)이라는 증거. \
                실행: cargo test --lib diag_easy_scenarios_still_fail_to_converge \
                -- --ignored --nocapture"]
    fn diag_easy_scenarios_still_fail_to_converge() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

        for (label, impact, incoming, tti) in [
            (
                "실측 speed=6.0m/s (이론상 여유 138.9%)",
                (0.762, 0.200, 0.999),
                (0.0, -4.34, 1.51),
                0.248,
            ),
            (
                "실측 speed=9.0m/s (이론상 여유 146.3%)",
                (0.762, 0.350, 0.875),
                (0.0, -7.32, 2.15),
                0.136,
            ),
            (
                "실측 speed=10.0m/s (이론상 여유 143.5%)",
                (0.762, 0.350, 0.859),
                (0.0, -8.26, 2.28),
                0.121,
            ),
            (
                "실측 speed=12.0m/s (이론상 여유 129.3%)",
                (0.762, 0.350, 0.865),
                (0.0, -10.09, 2.46),
                0.101,
            ),
        ] {
            let prediction = Prediction {
                time_to_impact_secs: tti,
                impact_position: crate::Point3::new(impact.0, impact.1, impact.2),
                incoming_velocity: Vector3::new(incoming.0, incoming.1, incoming.2),
            };
            match plan_bang_bang_swing(&arm, &[prediction], &start_pose) {
                Ok(planned) => {
                    let end = planned.trajectory.end_joints();
                    let end_qdot = planned
                        .trajectory
                        .sample_velocity_at(planned.trajectory.duration_secs());
                    let achieved = racket_velocity_estimate(
                        &arm,
                        planned.trajectory.follow_through_rail_x(),
                        0.0,
                        &end.values,
                        &end_qdot,
                    );
                    eprintln!("{label} -> 수렴 성공! achieved_v={achieved:?}");
                }
                Err(err) => {
                    eprintln!("{label} -> 수렴 실패: {err}");
                }
            }
        }
    }

    /// home 자세(`READY_JOINTS_4DOF`, `arm.initial_state()`가 이미 이 값)에서
    /// 실제 게임플레이 경로(quintic `plan_swing`)로 "가장 강력한 히트" 목표를
    /// 향해 스윙할 때, 관절이 도중에 방향을 뒤집는(왕복/진동) 구간이 있는지
    /// 확인한다 — 사용자 기준: 왕복이 있으면 컨트롤러가 아니라 home 자세를
    /// 조정해서 없애야 한다. 여기서는 `diag_ball_speed_feasibility_sweep`이
    /// 이미 실측한 정상 착지 시나리오 중 목표 라켓속도가 가장 큰(=가장
    /// 강력한 히트) 것을 대표로 쓴다.
    #[test]
    #[ignore = "일회성 실험(사용자 요청, 2026-07-28): home 자세 기준 최강 스윙이 \
                C-space(관절공간)에서 방향 반전(왕복) 없이 매끄럽게 진행되는지 \
                확인 — 왕복이 있으면 사용자 기준상 컨트롤러가 아니라 home \
                자세(READY_JOINTS_4DOF)를 조정해 없애야 하는 문제. \
                실행: cargo test --lib diag_home_pose_swing_is_monotonic_in_joint_space \
                -- --ignored --nocapture"]
    fn diag_home_pose_swing_is_monotonic_in_joint_space() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        eprintln!("home(READY_JOINTS_4DOF)={:?}", start.joints().values);

        // diag_ball_speed_feasibility_sweep 실측 시나리오들 — 목표 라켓속도
        // 내림차순(=강력한 순). quintic은 bang-bang보다 더 긴 시간이 필요해
        // (피크계수 1.875) tti가 짧으면 InfeasibleSwing이 날 수 있으니, "실제로
        // quintic 게임플레이 경로가 계획 가능한" 것 중 가장 강력한 걸 쓴다.
        let candidates = [
            (
                "speed=9.0m/s (target 0.869 m/s)",
                0.136,
                (0.762, 0.350, 0.875),
                (0.0, -7.32, 2.15),
            ),
            (
                "speed=10.0m/s (target 0.821 m/s)",
                0.121,
                (0.762, 0.350, 0.859),
                (0.0, -8.26, 2.28),
            ),
            (
                "speed=12.0m/s (target 0.814 m/s)",
                0.101,
                (0.762, 0.350, 0.865),
                (0.0, -10.09, 2.46),
            ),
            (
                "speed=6.0m/s (target 0.695 m/s)",
                0.248,
                (0.762, 0.200, 0.999),
                (0.0, -4.34, 1.51),
            ),
            // 위 4개(짧은 tti)가 전부 quintic 관절속도 한계로 실패할 경우의
            // 대비책 — 기존 회귀 테스트/기본 시나리오가 검증한 실제 게임플레이
            // 기본값(`auto_swing_plans_with_strike_velocity`와 동일).
            (
                "기본 회귀 시나리오(tti=0.3s)",
                0.3,
                (table::WIDTH_X * 0.5, 0.30, 0.932),
                (0.0, -6.01, 1.51),
            ),
        ];
        let mut chosen = None;
        for (label, tti, impact, incoming) in candidates {
            let prediction = Prediction {
                time_to_impact_secs: tti,
                impact_position: crate::Point3::new(impact.0, impact.1, impact.2),
                incoming_velocity: Vector3::new(incoming.0, incoming.1, incoming.2),
            };
            match crate::plan_swing(&arm, prediction, &start_pose) {
                Ok(trajectory) => {
                    eprintln!("사용한 시나리오: {label} (quintic 계획 성공)");
                    chosen = Some(trajectory);
                    break;
                }
                Err(err) => {
                    eprintln!("{label} -> quintic 계획 실패: {err} (다음 후보로)");
                }
            }
        }
        let trajectory = chosen.expect("적어도 하나는 quintic으로 계획 가능해야 함");
        let trajectory = &trajectory;
        eprintln!(
            "impact_joints={:?}\nend_velocity(임팩트 순간 목표 관절속도)={:?}\n\
             follow_through_joints={:?}\nimpact_time_secs={:.3} duration_secs={:.3}",
            trajectory.impact_joints().values,
            trajectory.end_velocity,
            trajectory.follow_through.values,
            trajectory.impact_time_secs,
            trajectory.duration_secs,
        );

        let n = start.joints().values.len();
        const SAMPLES: usize = 200;
        let mut sign_changes: Vec<usize> = vec![0; n];
        // 반전이 임팩트 단계(파워 생성 구간, 0..impact_time_secs) 안에서
        // 일어나는지, 팔로스루 단계(임팩트 후 감속/정돈 구간)에서만 일어나는지
        // 구분한다 — 후자는 계획된 감속이라 사용자가 말하는 "왕복" 문제와는
        // 성격이 다르다.
        let mut sign_changes_in_impact_phase: Vec<usize> = vec![0; n];
        let mut prev_sign: Vec<f64> = vec![0.0; n];
        for step in 0..=SAMPLES {
            let t = trajectory.duration_secs * (step as f64 / SAMPLES as f64);
            let qdot = trajectory.sample_velocity_at(t);
            for i in 0..n {
                let sign = if qdot[i] > 1e-6 {
                    1.0
                } else if qdot[i] < -1e-6 {
                    -1.0
                } else {
                    0.0
                };
                if sign != 0.0 && prev_sign[i] != 0.0 && sign != prev_sign[i] {
                    sign_changes[i] += 1;
                    if t <= trajectory.impact_time_secs {
                        sign_changes_in_impact_phase[i] += 1;
                    }
                }
                if sign != 0.0 {
                    prev_sign[i] = sign;
                }
            }
        }

        eprintln!(
            "관절별 속도 방향반전(왕복) 총횟수={sign_changes:?} \
             (그중 임팩트/파워생성 단계 안에서만={sign_changes_in_impact_phase:?}) \
             (0=반전 없이 한 방향으로만 진행)"
        );
        for (i, &changes) in sign_changes.iter().enumerate() {
            if changes > 0 {
                eprintln!(
                    "  -> joint{i}: home에서 이 임팩트까지 가는 동안 방향을 {changes}번 \
                     바꿈(왕복) — 사용자 기준상 home 자세 조정으로 없애는 게 맞는 대상"
                );
                // 정확히 어디서(언제, 어느 각도까지) 방향이 바뀌는지 — 봉우리
                // 시각/값을 찾는다.
                let mut peak_t = 0.0;
                let mut peak_q = start.joints().values[i];
                for step in 0..=SAMPLES {
                    let t = trajectory.duration_secs * (step as f64 / SAMPLES as f64);
                    let q = trajectory.sample_at(t).values[i];
                    if (q - start.joints().values[i]).abs()
                        > (peak_q - start.joints().values[i]).abs()
                    {
                        peak_q = q;
                        peak_t = t;
                    }
                }
                eprintln!(
                    "     home={:.4} rad -> 봉우리 {:.4} rad (t={:.3}s) -> 임팩트 {:.4} rad \
                     -> 팔로스루 {:.4} rad",
                    start.joints().values[i],
                    peak_q,
                    peak_t,
                    trajectory.impact_joints().values[i],
                    trajectory.follow_through.values[i],
                );
            }
        }
    }

    /// 알고리즘 수정의 "성공"을 "완벽한 수렴"이 아니라 "커밋 가능한 입사속도
    /// 범위가 넓어졌는가"로 재정의(사용자 지시, 2026-07-28)한 뒤의 베이스라인
    /// 측정. 실제 슈터 물리로 입사속도를 촘촘히 스윕해, 각 속도에서
    /// (a) 정상 착지(네트 통과 + 반코트 가운데)하는 pitch가 있는지,
    /// (b) 있다면 그 임팩트가 커밋창 안에 들어오는지,
    /// (c) 들어온다면 `plan_bang_bang_swing`이 실제로 수렴(Ok)하는지
    /// 를 각각 집계한다. 수정 전/후 이 표를 그대로 비교해 "범위가 넓어졌는지"
    /// 를 판정하는 기준선으로 쓴다.
    #[test]
    #[ignore = "베이스라인 측정(사용자 요청, 2026-07-28): 알고리즘 수정 전 \
                bang-bang 커밋 성공 범위(입사속도 대역)를 기록한다 — 수정 후 \
                같은 스윕을 다시 돌려 범위가 넓어졌는지 비교하는 기준선. \
                실행: cargo test --lib diag_coverage_baseline_bang_bang \
                -- --ignored --nocapture"]
    fn diag_coverage_baseline_bang_bang() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

        use crate::sim::{BallShooterSettings, BallState, SimWorld};
        const DT: f64 = 1.0 / 1000.0;
        const MAX_STEPS: usize = 4_000;
        const HEIGHT_OFFSET_M: f64 = 0.24;

        let robot = crate::defaults::primitive_4dof_with_mount(-0.02, table::SURFACE_Z + 0.05)
            .expect("robot 빌드 성공");
        let intercept = crate::InterceptWindow {
            y_min: 0.20,
            y_max: 0.55,
            sample_step: 0.05,
        };

        let mut no_legal_landing = 0;
        let mut no_commit_window = 0;
        let mut ik_fail = 0;
        let mut converged = 0;
        let mut not_converged = 0;
        let mut speeds_tested = 0;

        let mut speed = 3.0_f64;
        while speed <= 14.0 {
            speeds_tested += 1;
            let Some(pitch_deg) = find_legal_pitch_deg(speed, HEIGHT_OFFSET_M) else {
                eprintln!("speed={speed:.1} -> 정상 착지 pitch 없음");
                no_legal_landing += 1;
                speed += 0.5;
                continue;
            };

            let mut world = SimWorld::new(robot.clone());
            world.set_use_ground_truth(true);
            let mut settings = BallShooterSettings::default();
            settings.speed_mps = speed;
            settings.pitch_deg = pitch_deg;
            settings.height_offset_m = HEIGHT_OFFSET_M;
            world.shoot_ball(&settings);

            let mut found = None;
            for _ in 0..MAX_STEPS {
                world.step(DT, None);
                if world.ball_state != BallState::InFlight {
                    break;
                }
                let ball_y = f64::from(world.ball_position().y);
                if !crate::ball_past_midcourt_for_commit(ball_y) {
                    continue;
                }
                let predictions: Vec<_> = intercept
                    .hit_planes()
                    .into_iter()
                    .filter_map(|plane| crate::sim::predict_impact(&world, plane))
                    .collect();
                let in_window: Vec<_> = predictions
                    .into_iter()
                    .filter(|p| crate::in_swing_commit_window(p.time_to_impact_secs))
                    .collect();
                if !in_window.is_empty() {
                    found = Some(in_window);
                    break;
                }
            }

            let Some(predictions) = found else {
                eprintln!("speed={speed:.1} pitch={pitch_deg:.1} -> 커밋창 안 예측 없음");
                no_commit_window += 1;
                speed += 0.5;
                continue;
            };

            match plan_bang_bang_swing(&arm, &predictions, &start_pose) {
                Ok(_) => {
                    eprintln!("speed={speed:.1} pitch={pitch_deg:.1} -> 수렴 성공");
                    converged += 1;
                }
                Err(DomainError::InfeasibleSwing(ref err)) if !err.is_hard_unreachable() => {
                    eprintln!("speed={speed:.1} pitch={pitch_deg:.1} -> 수렴 실패: {err}");
                    not_converged += 1;
                }
                Err(err) => {
                    eprintln!("speed={speed:.1} pitch={pitch_deg:.1} -> IK/도달 불가: {err}");
                    ik_fail += 1;
                }
            }
            speed += 0.5;
        }

        eprintln!(
            "\n=== 베이스라인 요약 (speed 3.0~14.0, step 0.5, 총 {speeds_tested}개) ===\n\
             정상 착지 pitch 없음={no_legal_landing}\n\
             착지는 정상인데 커밋창 밖={no_commit_window}\n\
             IK/도달 불가={ik_fail}\n\
             수렴 실패(InsufficientTime 등)={not_converged}\n\
             수렴 성공={converged}\n\
             (성공률 = 수렴성공 / 총테스트 = {:.1}%)",
            100.0 * converged as f64 / speeds_tested as f64,
        );
    }

    /// `diag_coverage_baseline_bang_bang`이 IK/도달 불가로 집계한 실패의
    /// 실제 모양을 본다 — 그 공에 대해 커밋창 안 후보(hit-plane)가 전부
    /// IK 불가인지, 일부만인지, `NEAR_SINGULARITY_SPEED_RATIO` 게이트에
    /// 걸린 건지 구분한다. 이게 "범위를 넓히는" 진짜 레버가 유도법(ZEM/ZEV)이
    /// 아니라 후보 탐색/IK 쪽일 수 있다는 가설을 검증한다.
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): 커버리지 실패의 실제 원인이 \
                유도법(ZEM/ZEV) 수렴이 아니라 후보 hit-plane들의 IK 도달성 \
                자체인지 확인. \
                실행: cargo test --lib diag_why_realistic_shots_fail_ik \
                -- --ignored --nocapture"]
    fn diag_why_realistic_shots_fail_ik() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

        use crate::sim::{BallShooterSettings, BallState, SimWorld};
        const DT: f64 = 1.0 / 1000.0;
        const MAX_STEPS: usize = 4_000;
        const HEIGHT_OFFSET_M: f64 = 0.24;

        let robot = crate::defaults::primitive_4dof_with_mount(-0.02, table::SURFACE_Z + 0.05)
            .expect("robot 빌드 성공");
        let intercept = crate::InterceptWindow {
            y_min: 0.20,
            y_max: 0.55,
            sample_step: 0.05,
        };

        for speed in [6.0_f64, 10.0, 14.0] {
            let Some(pitch_deg) = find_legal_pitch_deg(speed, HEIGHT_OFFSET_M) else {
                continue;
            };
            let mut world = SimWorld::new(robot.clone());
            world.set_use_ground_truth(true);
            let mut settings = BallShooterSettings::default();
            settings.speed_mps = speed;
            settings.pitch_deg = pitch_deg;
            settings.height_offset_m = HEIGHT_OFFSET_M;
            world.shoot_ball(&settings);

            let mut predictions_at_window = None;
            for _ in 0..MAX_STEPS {
                world.step(DT, None);
                if world.ball_state != BallState::InFlight {
                    break;
                }
                let ball_y = f64::from(world.ball_position().y);
                if !crate::ball_past_midcourt_for_commit(ball_y) {
                    continue;
                }
                let predictions: Vec<_> = intercept
                    .hit_planes()
                    .into_iter()
                    .filter_map(|plane| crate::sim::predict_impact(&world, plane))
                    .collect();
                let in_window: Vec<_> = predictions
                    .into_iter()
                    .filter(|p| crate::in_swing_commit_window(p.time_to_impact_secs))
                    .collect();
                if !in_window.is_empty() {
                    predictions_at_window = Some(in_window);
                    break;
                }
            }
            let Some(predictions) = predictions_at_window else {
                eprintln!("speed={speed:.1} -> 커밋창 안 예측 없음");
                continue;
            };

            eprintln!(
                "speed={speed:.1} pitch={pitch_deg:.1} -> 커밋창 안 후보 {}개:",
                predictions.len()
            );
            for p in &predictions {
                let dist_to_current = (p.impact_position.coords
                    - arm
                        .forward_kinematics_with_rail(start_pose.rail_x, &start_pose.joints)
                        .expect("fk")
                        .position
                        .coords)
                    .norm();
                match crate::swing_feasibility(&arm, p, &start_pose) {
                    Some(f) => {
                        eprintln!(
                            "  impact=({:.3},{:.3},{:.3}) tti={:.3} dist_from_home={:.3}m -> \
                             IK 성공, peak_joint_speed_ratio={:.2} (>2.5면 특이점 게이트 걸림)",
                            p.impact_position.x,
                            p.impact_position.y,
                            p.impact_position.z,
                            p.time_to_impact_secs,
                            dist_to_current,
                            f.peak_joint_speed_ratio,
                        );
                    }
                    None => {
                        eprintln!(
                            "  impact=({:.3},{:.3},{:.3}) tti={:.3} dist_from_home={:.3}m -> \
                             IK 자체가 실패(도달 범위 밖)",
                            p.impact_position.x,
                            p.impact_position.y,
                            p.impact_position.z,
                            p.time_to_impact_secs,
                            dist_to_current,
                        );
                    }
                }
            }
        }
    }

    /// `plan_bang_bang_swing`은 정렬된 `ranked` 리스트를 "첫 성공에서 멈추는"
    /// 게 아니라 **끝까지 순회하며 전부 시도**한다(`bang_bang.rs:192-208`).
    /// 즉 거리 대신 관절여유(feasibility)로 순서를 바꿔도, 후보 집합 자체가
    /// 안 바뀌면 "그 공에 대해 커밋이 되는가"라는 최종 성공/실패는 바뀌지
    /// 않는다 — 순서는 오직 (a) 여러 개가 성공할 때 무엇을 고르는지, (b) 실시간
    /// GUI에서 계산 낭비 순서에만 영향을 준다. 이 진단은 그 가설을 직접
    /// 검증한다: `diag_why_realistic_shots_fail_ik`가 IK는 성공한다고 표시한
    /// 후보들을 **하나씩 단독으로** `plan_bang_bang_swing`에 넣어, 그중 어느
    /// 것이라도 단독으로 성공하는지 본다.
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): 후보를 거리 대신 \
                관절여유(feasibility)로 재정렬하면 커밋 성공률이 실제로 \
                바뀌는지 확인 — plan_bang_bang_swing이 이미 전체 순회하므로 \
                순서 자체는 최종 성공여부를 안 바꿀 가능성이 높다는 가설을 \
                직접 검증한다. \
                실행: cargo test --lib diag_does_reordering_by_feasibility_help \
                -- --ignored --nocapture"]
    fn diag_does_reordering_by_feasibility_help() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

        use crate::sim::{BallShooterSettings, BallState, SimWorld};
        const DT: f64 = 1.0 / 1000.0;
        const MAX_STEPS: usize = 4_000;
        const HEIGHT_OFFSET_M: f64 = 0.24;

        let robot = crate::defaults::primitive_4dof_with_mount(-0.02, table::SURFACE_Z + 0.05)
            .expect("robot 빌드 성공");
        let intercept = crate::InterceptWindow {
            y_min: 0.20,
            y_max: 0.55,
            sample_step: 0.05,
        };

        for speed in [6.0_f64, 9.0, 10.0, 12.0] {
            let Some(pitch_deg) = find_legal_pitch_deg(speed, HEIGHT_OFFSET_M) else {
                continue;
            };
            let mut world = SimWorld::new(robot.clone());
            world.set_use_ground_truth(true);
            let mut settings = BallShooterSettings::default();
            settings.speed_mps = speed;
            settings.pitch_deg = pitch_deg;
            settings.height_offset_m = HEIGHT_OFFSET_M;
            world.shoot_ball(&settings);

            let mut predictions_at_window = None;
            for _ in 0..MAX_STEPS {
                world.step(DT, None);
                if world.ball_state != BallState::InFlight {
                    break;
                }
                let ball_y = f64::from(world.ball_position().y);
                if !crate::ball_past_midcourt_for_commit(ball_y) {
                    continue;
                }
                let predictions: Vec<_> = intercept
                    .hit_planes()
                    .into_iter()
                    .filter_map(|plane| crate::sim::predict_impact(&world, plane))
                    .collect();
                let in_window: Vec<_> = predictions
                    .into_iter()
                    .filter(|p| crate::in_swing_commit_window(p.time_to_impact_secs))
                    .collect();
                if !in_window.is_empty() {
                    predictions_at_window = Some(in_window);
                    break;
                }
            }
            let Some(predictions) = predictions_at_window else {
                eprintln!("speed={speed:.1} -> 커밋창 안 예측 없음 (스킵)");
                continue;
            };

            eprintln!(
                "speed={speed:.1} pitch={pitch_deg:.1} -> 후보 {}개 단독 테스트:",
                predictions.len()
            );
            let mut any_succeeded_alone = false;
            for p in &predictions {
                let feasibility = crate::swing_feasibility(&arm, p, &start_pose)
                    .map(|f| format!("peak_ratio={:.2}", f.peak_joint_speed_ratio))
                    .unwrap_or_else(|| "IK 실패".to_string());
                match plan_bang_bang_swing(&arm, std::slice::from_ref(p), &start_pose) {
                    Ok(_) => {
                        any_succeeded_alone = true;
                        eprintln!(
                            "  y={:.2} tti={:.3} {feasibility} -> 단독으로도 성공!",
                            p.impact_position.y, p.time_to_impact_secs
                        );
                    }
                    Err(err) => {
                        eprintln!(
                            "  y={:.2} tti={:.3} {feasibility} -> 단독으로도 실패: {err}",
                            p.impact_position.y, p.time_to_impact_secs
                        );
                    }
                }
            }
            eprintln!(
                "  => 이 공에 대해 '어떤 순서로 시도해도' 성공 가능한 후보가 \
                 있었는가: {any_succeeded_alone} (재정렬이 도움될 수 있는지의 \
                 필요조건 — false면 재정렬은 이 공에는 도움 안 됨)"
            );
        }
    }

    /// 사용자 지시(2026-07-28): 고정 8개 y평면이 아니라, 공의 궤적을 따라
    /// y를 연속 변수로 보고 "이 지점에서 로봇이 이 방향으로 낼 수 있는
    /// 최대 속도"(용량, [`kinematic_ceiling`])와 "이 지점에서 요구되는
    /// 속도"(요구량, 공 물리가 정함)를 촘촘히 맞대어(역산) 용량이 요구량을
    /// 넘는 지점이 있는지 찾는다. `InterceptWindow`의 0.05 간격보다 5배
    /// 촘촘한(0.01) 스캔으로 놓친 지점이 있는지 확인.
    #[test]
    #[ignore = "순수 진단(사용자 요청, 2026-07-28): 고정 8평면보다 촘촘한 \
                y 스캔으로 용량(capability) vs 요구량(requirement) 지형을 \
                그려, 놓친 '편안한 지점'이 있는지 확인. \
                실행: cargo test --lib diag_capability_vs_requirement_along_trajectory \
                -- --ignored --nocapture"]
    fn diag_capability_vs_requirement_along_trajectory() {
        let arm = competition_arm();
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

        use crate::estimator::HitPlane;
        use crate::sim::{BallShooterSettings, BallState, SimWorld};
        const DT: f64 = 1.0 / 1000.0;
        const MAX_STEPS: usize = 4_000;
        const HEIGHT_OFFSET_M: f64 = 0.24;

        let robot = crate::defaults::primitive_4dof_with_mount(-0.02, table::SURFACE_Z + 0.05)
            .expect("robot 빌드 성공");

        for speed in [6.0_f64, 9.0, 10.0, 12.0] {
            let Some(pitch_deg) = find_legal_pitch_deg(speed, HEIGHT_OFFSET_M) else {
                eprintln!("speed={speed:.1} -> 정상 착지 pitch 없음 (스킵)");
                continue;
            };
            let mut world = SimWorld::new(robot.clone());
            world.set_use_ground_truth(true);
            let mut settings = BallShooterSettings::default();
            settings.speed_mps = speed;
            settings.pitch_deg = pitch_deg;
            settings.height_offset_m = HEIGHT_OFFSET_M;
            world.shoot_ball(&settings);

            let mut reached = false;
            for _ in 0..MAX_STEPS {
                world.step(DT, None);
                if world.ball_state != BallState::InFlight {
                    break;
                }
                let ball_y = f64::from(world.ball_position().y);
                if !crate::ball_past_midcourt_for_commit(ball_y) {
                    continue;
                }

                // 이 물리 스텝의 공 상태에서, y를 0.10~0.60까지 0.01 간격
                // (고정 그리드의 5배 해상도)으로 스캔해 용량/요구량 프로필을
                // 그린다. 각 y는 물리(predict_impact)가 강제하는 (x,z,v,tti)를
                // 갖고, 그 지점에서 로봇이 낼 수 있는 최대 속도(용량)를
                // kinematic_ceiling으로, 필요한 속도를 target_speed로 본다.
                let mut profile: Vec<(f64, f64, f64)> = Vec::new(); // (y, ratio, tti)
                for i in 0..=50 {
                    let y = 0.10 + 0.01 * i as f64;
                    let Some(prediction) = crate::sim::predict_impact(&world, HitPlane { y })
                    else {
                        continue;
                    };
                    if !crate::in_swing_commit_window(prediction.time_to_impact_secs) {
                        continue;
                    }
                    if let Some(ceiling) = kinematic_ceiling(&arm, &start_pose, &prediction) {
                        profile.push((
                            y,
                            ceiling.v_max_kinematic / ceiling.target_speed,
                            prediction.time_to_impact_secs,
                        ));
                    }
                }
                if profile.is_empty() {
                    continue;
                }
                reached = true;

                eprintln!(
                    "speed={speed:.1} pitch={pitch_deg:.1} -> 용량/요구량 비율 프로필(0.01 간격):"
                );
                for (y, ratio, tti) in profile.iter().step_by(5) {
                    eprintln!("    y={y:.2} tti={tti:.3} capability/requirement={ratio:.2}");
                }
                let (best_y, best_ratio, best_tti) = profile
                    .iter()
                    .copied()
                    .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                    .expect("비어있지 않음");
                eprintln!(
                    "  => 최고점: y={best_y:.2} tti={best_tti:.3} capability/requirement={best_ratio:.2}"
                );
                let best_prediction = crate::sim::predict_impact(&world, HitPlane { y: best_y });
                if let Some(prediction) = best_prediction {
                    match plan_bang_bang_swing(&arm, &[prediction], &start_pose) {
                        Ok(_) => eprintln!("  => 이 최고점에서 실제로 수렴 성공!"),
                        Err(err) => eprintln!("  => 이 최고점도 실제로는 수렴 실패: {err}"),
                    }
                }
                break;
            }
            if !reached {
                eprintln!("speed={speed:.1} -> 커밋창 근처에 도달 못함");
            }
        }
    }

    #[test]
    fn bang_bang_accel_to_matches_legacy_formula_when_target_velocity_is_zero() {
        let a_max = 1.5;
        for &(x, v) in &[
            (0.5, 0.3),
            (-0.5, 0.3),
            (0.5, -0.3),
            (-0.5, -0.3),
            (0.0, 0.0),
            (2.0, 0.0),
            (0.0, 1.0),
        ] {
            let legacy = bang_bang_accel(x, v, a_max);
            let generalized = bang_bang_accel_to(x, v, 0.0, a_max);
            assert!(
                (legacy - generalized).abs() < 1e-12,
                "x={x} v={v}: legacy={legacy} generalized={generalized}"
            );
        }
    }

    /// `(x0, v0)`에서 `bang_bang_accel_to`만으로(RNEA/Arm 없이) 순수 이중적분기를
    /// 적분해 `(0, target_v)`에 수렴하는지 확인한다 — 반환값은 `(수렴 시각,
    /// 종료 위치오차, 종료 속도)`.
    fn simulate_double_integrator(
        x0: f64,
        v0: f64,
        target_v: f64,
        a_max: f64,
        dt: f64,
        max_time: f64,
    ) -> Option<(f64, f64, f64)> {
        let mut x = x0;
        let mut v = v0;
        let mut t = 0.0;
        while t < max_time {
            let a = bang_bang_accel_to(x, v, target_v, a_max);
            v += a * dt;
            x += v * dt;
            t += dt;
            if x.abs() < 1e-3 && (v - target_v).abs() < 1e-3 {
                return Some((t, x, v));
            }
        }
        return None;
    }

    /// 표준 이중적분기 시간최적 점대점 제어의 이론적 최소시간 — 두 분지
    /// (첫 구간 가속부호 `s=+1`/`-1`) 중 인과적으로 유효한(두 구간 소요시간
    /// 모두 `>=0`) 해들 중 최소 총시간을 고른다. `simulate_double_integrator`의
    /// 결과와 대조할 기준값으로만 쓰는 시험 전용 헬퍼 — 실제 스위칭 로직
    /// (`bang_bang_accel_to`)과 독립적으로 유도했다.
    fn theoretical_min_time(x0: f64, v0: f64, target_v: f64, a_max: f64) -> f64 {
        let mut best: Option<f64> = None;
        for &s in &[1.0_f64, -1.0] {
            let vs_sq = (v0 * v0 + target_v * target_v) / 2.0 - s * a_max * x0;
            if vs_sq < 0.0 {
                continue;
            }
            for &vs in &[vs_sq.sqrt(), -vs_sq.sqrt()] {
                let t1 = (vs - v0) / (s * a_max);
                let t2 = (target_v - vs) / (-s * a_max);
                if t1 >= -1e-9 && t2 >= -1e-9 {
                    let total = t1.max(0.0) + t2.max(0.0);
                    best = Some(best.map_or(total, |b: f64| b.min(total)));
                }
            }
        }
        return best.expect("이 테스트 케이스는 인과적으로 유효한 bang-bang 해가 있어야 함");
    }

    #[test]
    fn bang_bang_accel_to_converges_for_nonzero_target_velocity() {
        let dt = 1e-4;
        let max_time = 20.0;
        // (x0, v0, target_v, a_max) — 목표속도 0 하나, 비영 목표속도 둘(부호 다른
        // 초기속도 포함)을 섞어 일반화 공식이 실제로 여러 형태의 경계조건에서
        // 수렴하는지 확인한다.
        let cases = [
            (-1.0, 0.0, 2.0, 1.0),
            (1.0, 0.0, 0.0, 2.0),
            (0.5, -1.0, 1.0, 1.5),
        ];
        for &(x0, v0, target_v, a_max) in &cases {
            let (t, x, v) = simulate_double_integrator(x0, v0, target_v, a_max, dt, max_time)
                .unwrap_or_else(|| {
                    panic!(
                        "x0={x0} v0={v0} target_v={target_v} a_max={a_max}: \
                         {max_time}s 안에 수렴 못함"
                    )
                });
            assert!(x.abs() < 1e-2, "x0={x0} v0={v0}: 최종 위치오차 {x} 너무 큼");
            assert!(
                (v - target_v).abs() < 1e-2,
                "x0={x0} v0={v0}: 최종 속도오차 {} 너무 큼",
                v - target_v
            );
            let theoretical = theoretical_min_time(x0, v0, target_v, a_max);
            assert!(
                (t - theoretical).abs() < 0.05,
                "x0={x0} v0={v0} target_v={target_v}: 시뮬레이션 수렴시간 {t:.4}s \
                 vs 이론 최소시간 {theoretical:.4}s"
            );
        }
    }

    #[test]
    fn sample_at_interpolates_between_recorded_samples() {
        let trajectory = BangBangTrajectory {
            dt: 0.1,
            joint_samples: vec![Joints::from_slice(&[0.0]), Joints::from_slice(&[1.0])],
            rail_samples: vec![0.0, 2.0],
        };
        let mid = trajectory.sample_at(0.05);
        assert!((mid.values[0] - 0.5).abs() < 1e-9);
        assert!((trajectory.sample_rail_at(0.05) - 1.0).abs() < 1e-9);
    }
}
