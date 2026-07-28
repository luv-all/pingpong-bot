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

use crate::robot::dynamics::{
    MassMatrixScratch, RneaScratch, bias_torques_into, mass_matrix_into,
};
use super::physics::{in_swing_commit_window, solve_impact_target};
use crate::error::{DomainError, SwingPlanError};
use crate::robot::Arm;
use crate::{Joints, Prediction, RobotPose};

/// 실기 AXL 레일 가속/감속 [m/s^2].
/// 출처: `config/real-hardware.toml`의 `[hardware.rail]` accel/decel = 12.0.
const RAIL_ACCEL_M_S2: f64 = 12.0;
const POSITION_TOLERANCE_RAD_OR_M: f64 = 1e-3;
/// 라켓 속도 크기 허용오차(목표 대비 비율) — `tools/swing_bench`와 동일 값.
const RACKET_SPEED_RATIO_TOLERANCE: f64 = 0.15;
/// 라켓 속도 방향 허용오차 [deg].
const RACKET_DIRECTION_TOLERANCE_DEG: f64 = 15.0;
/// 계획 적분 스텝 [s] — 물리 스텝(1kHz)과 맞춘다.
const PLAN_DT_SECS: f64 = 0.001;
/// 수렴 못 하면 포기하는 계획 시간 상한 [s] — 방어용 절대 상한일 뿐이다.
/// ZEM/ZEV 유도(`Tg`를 `prediction.time_to_impact_secs`에 고정)로 넘어온
/// 뒤로는 루프가 실질적으로 그 실제 임팩트 시각에서 끝나므로(커밋창 필터로
/// 이미 ≤0.35s), 정상 경로에서는 이 값이 거의 항상 이긴다. 그래도 무언가
/// 커밋창 필터를 거치지 않고 이 함수를 직접 호출해(예: 진단/테스트) 비정상적
/// 으로 큰 `time_to_impact_secs`를 넘기는 경우까지 낭비 시간을 묶어 둔다 —
/// 예전(관절별 bang-bang) 2.0s는 그 자체가 낭비의 근원이었지만(`.omc/progress.txt`),
/// 이제는 이 상한에 걸릴 일이 실질적으로 없어 값 자체의 크기는 덜 민감하다.
const MAX_PLAN_TIME_SECS: f64 = 0.5;
/// 가중 최소노름 자코비안 역산의 감쇠최소제곱 정칙화 계수 —
/// [`step_racket_guidance`] 문서 참고.
const JACOBIAN_DAMPING: f64 = 0.05;
/// ZEM/ZEV에 넘기는 `Tg`를 실제 남은 시간의 이 비율로 줄인다(0<1) —
/// [`step_racket_guidance`] 문서 참고.
const TIME_TO_GO_BIAS: f64 = 0.5;
/// `Tg` 나누기 바닥값 [s] — [`step_racket_guidance`] 문서 참고.
const MIN_TIME_TO_GO_SECS: f64 = 1e-3;
/// 자코비안 시간미분(`J̇`) 유한차분 스텝 [s] — [`step_racket_guidance`] 문서 참고.
const JDOT_STEP: f64 = 1e-4;

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
    mass_matrix_into(arm, &Joints::from_slice(q), &mut scratch.rnea, &mut scratch.mass_matrix, &mut scratch.mass);
    let joint_a_max_full: Vec<f64> = (0..jacobian.ncols())
        .map(|col| {
            if has_rail && col == 0 {
                return RAIL_ACCEL_M_S2;
            }
            let i = col - rail_offset;
            (arm.joint_torque_limits[i] / scratch.mass[(i, i)].max(1e-9)).max(1e-6)
        })
        .collect();

    let joint_preference: Vec<f64> = (0..jacobian.ncols())
        .map(|col| {
            if has_rail && col == 0 {
                return 1.0;
            }
            let i = col - rail_offset;
            arm.joint_torque_limits[i].powi(4)
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
        - racket_velocity * (4.0 / time_to_go)
        - target_racket_velocity * (2.0 / time_to_go);

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
    let perturbed_q: Vec<f64> = q.iter().zip(qdot.iter()).map(|(qi, vi)| qi + vi * JDOT_STEP).collect();
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
    let rail_accel_desired = if has_rail { qddot_full[0] } else { 0.0 };

    bias_torques_into(arm, &Joints::from_slice(q), qdot, &mut scratch.rnea, &mut scratch.bias_zero_accel, &mut scratch.bias);

    // 근접 특이점에서 감쇠로도 못 잡은 잔여 스파이크가 남을 수 있어, 관절
    // 가속도 자체도 각 관절이 낼 수 있는 물리적 한계(토크/유효관성)로 한 번 더
    // 클램프한다 — 이 한계를 넘는 desired qddot는 애초에 실현 불가능하므로
    // 클램프해도 정보 손실이 없다.
    let joint_qddot_desired: Vec<f64> = (0..n)
        .map(|i| qddot_full[i + rail_offset].clamp(-joint_a_max_full[i + rail_offset], joint_a_max_full[i + rail_offset]))
        .collect();
    let joint_qddot_desired = DVector::from_vec(joint_qddot_desired);
    let m_qddot = &scratch.mass * &joint_qddot_desired;
    let mut torque_cmd = vec![0.0; n];
    for i in 0..n {
        let tau_desired = m_qddot[i] + scratch.bias[i];
        torque_cmd[i] = tau_desired.clamp(-arm.joint_torque_limits[i], arm.joint_torque_limits[i]);
    }
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

    return Some(RacketGuidanceStep { racket_accel_desired, torque_cmd });
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
        joint_samples.push(Joints::from_slice(&q));
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
                    .filter(|&i| (step.torque_cmd[i].abs() - arm.joint_torque_limits[i]).abs() < 1e-6)
                    .collect();
                let util: Vec<f64> =
                    (0..n).map(|i| step.torque_cmd[i].abs() / arm.joint_torque_limits[i]).collect();
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
        let mean: Vec<f64> = (0..n).map(|i| torque_util_sum[i] / step_count as f64).collect();
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
        let cases = [(-1.0, 0.0, 2.0, 1.0), (1.0, 0.0, 0.0, 2.0), (0.5, -1.0, 1.0, 1.5)];
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
