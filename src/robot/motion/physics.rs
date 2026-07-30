//! 순수 물리/스윙 계획.

use nalgebra::Vector3;

use crate::constants::table;
use crate::defaults;
use crate::defaults::motion::{
    RETURN_TO_CENTER_GROWTH, RETURN_TO_CENTER_MAX_SECS, RETURN_TO_CENTER_MIN_SECS,
};
use crate::error::{DomainError, SwingPlanError};
use crate::estimator::Impact;
use crate::estimator::Prediction;
use crate::robot::Arm;
use crate::robot::{self, Joints};

use super::impact_target::solve_impact_target;
use super::planned_intercept::PlannedIntercept;
use super::rail::Rail;
use super::trajectory::Trajectory;

/// 임팩트까지 남은 시간이 스윙 commit 창 `[MIN_SWING, COMMIT_MAX]` 안인지.
///
/// 창보다 이르면 대기(발사 직후 긴 궤적 금지), 짧으면 `InsufficientTime`.
pub fn in_swing_commit_window(time_to_impact_secs: f64) -> bool {
    return (defaults::ControlParams::default().min_swing_secs
        ..=defaults::ControlParams::default().swing_commit_max_secs)
        .contains(&time_to_impact_secs);
}

/// 네트 통과 후인지 - ground truth/EKF control 공통 commit 게이트.
pub fn ball_past_midcourt_for_commit(ball_y: f64) -> bool {
    return ball_y
        <= table::LENGTH_Y * defaults::ControlParams::default().swing_commit_max_ball_y_frac;
}

/// 예측/현재 포즈로 quintic 스윙 궤적을 계획한다.
pub fn plan_swing(
    arm: &Arm,
    prediction: Prediction,
    start: &robot::Pose,
) -> Result<Trajectory, DomainError> {
    let time_to_impact = prediction.time_to_impact_secs;
    if time_to_impact < defaults::ControlParams::default().min_swing_secs {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::InsufficientTime {
                time_to_impact_secs: time_to_impact,
                min_swing_secs: defaults::ControlParams::default().min_swing_secs,
            },
        ));
    }

    let target = solve_impact_target(arm, &prediction, start)?;

    let start_velocity = vec![0.0; start.joints.values.len()];
    let rail_motion = Rail {
        start: start.rail_x,
        end: target.pose.rail_x,
        start_velocity: 0.0,
        end_velocity: target.rail_velocity,
    };

    return build_feasible_trajectory(
        arm,
        &start.joints,
        target.pose.joints,
        start_velocity,
        target.joint_velocities,
        time_to_impact,
        rail_motion,
    )
    .map_err(DomainError::InfeasibleSwing);
}

/// 계획 접촉점과 예측 임팩트점의 허용 오차 [m] — 넘으면 후보를 통째로 기각.
///
/// WP2c(2026-07-29~30) 실측(`docs/wp2c-contact-tolerance.md`): 값을
/// 5e-5~0.055 m 범위(1100배)로 스윕해도 커밋률·eval 점수·실제 접촉 좌표가
/// **완전히 동일** — 이 게이트는 현재 커밋률의 병목이 아니다. 이유는
/// 해석적으로 막혀 있기 때문이다: `solve_impact_target`의 IK 수렴 오차
/// (위치 2e-4 m + 법선 1e-3 rad × (BALL_RADIUS+RACKET_HALF_Z))가 상한
/// 2.25e-4 m을 만들고, 실측 최대 채택 오차는 0.00017 m로 그 아래다 — 즉
/// 5mm는 IK가 실제로 낼 수 있는 오차의 22배 여유를 둔 값이다.
///
/// 두 방향 경계: 아래로는 IK 수렴 오차(2.25e-4 m)보다 커야 유효한 해를
/// 기각하지 않고, 위로는 라켓 면 반너비 − 공 반지름(≈0.055 m)보다 작아야
/// 접촉이 면 안에 확실히 들어온다. 현재값은 그 244배 폭 구간의 아래쪽
/// (하한의 22배, 상한의 1/11)에 있다 — 이 상수는 필터가 아니라 계약
/// 위반 감지용 트립와이어라, 폭이 남아도 좁게 유지하는 쪽이 맞다(IK 품질
/// 회귀를 조기에 잡음).
pub const MAX_CONTACT_ERROR: f64 = 0.005;

pub fn plan_best_swing(
    arm: &Arm,
    predictions: &[Prediction],
    start: &robot::Pose,
) -> Result<PlannedIntercept, DomainError> {
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
        let trajectory = match plan_swing(arm, prediction, start) {
            Ok(trajectory) => trajectory,
            Err(error) => {
                last_error = Some(error);
                continue;
            }
        };
        let pose = if arm.rail.is_some() {
            arm.forward_kinematics_with_rail(trajectory.rail.end, &trajectory.end)
        } else {
            arm.forward_kinematics(&trajectory.end)
        };
        let Some(pose) = pose else {
            continue;
        };
        let contact = pose.position.coords
            + pose.normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z);
        if (contact - prediction.impact_position.coords).norm() > MAX_CONTACT_ERROR {
            continue;
        }
        return Ok(PlannedIntercept {
            prediction,
            trajectory,
        });
    }
    return Err(last_error.unwrap_or(DomainError::InfeasibleSwing(
        SwingPlanError::InverseKinematicsNoSolution {
            target_x: 0.0,
            target_y: 0.0,
            target_z: 0.0,
        },
    )));
}

/// commit 전 값싼 rough 추종용 목표 포즈를 계산한다 (rough-to-fine의 rough).
///
/// 아직 공이 네트를 안 넘어 탄도가 안정되기 전 단계에서, 레일/관절을 예측
/// 임팩트 쪽으로 미리 옮겨 두기 위한 best-effort 목표다. `plan_best_swing`의
/// 다중 평면 랭킹·전 궤적 충돌 샘플링은 하지 않는다 (그건 commit 단계 몫).
///
/// 가장 임박한(time_to_impact 최소) 예측 하나만 골라 단일 IK 호출
/// (`inverse_pose_with_rail`)로 rough 포즈를 구한다. IK가 수렴 못 하면 `None`
/// — 확정 스윙이 아니라 rough 목표라 실패는 에러가 아니라 "이번 틱 스킵"이다.
pub fn plan_coarse_track(arm: &Arm, predictions: &[Prediction]) -> Option<robot::Pose> {
    let target = coarse_track_geometry(arm, predictions)?;
    let rail = arm.rail.as_ref()?;
    // 기본 중앙 포즈를 힌트로 단일 IK. 실제 이동은 rate-limited 추종 루프가 함.
    let hint = robot::Pose::new(rail.default_x(), arm.default_joints.clone());
    return arm
        .inverse_pose_with_rail(target.reachable, target.desired_normal, &hint)
        .ok();
}

/// coarse 추종 목표를 **한 번의 기하 계산으로** 레일 x와 회전 관절 목표를 함께
/// 낸다 — 시뮬 루프처럼 둘 다 필요한 호출자용.
///
/// 레일 x만 따로 구하는 함수와 [`plan_coarse_track`]을 각각 부르면 예측
/// 선택·반사 법선 기하를 두 번 돌게 되고, 이 경로는 **매 물리 틱**(1 kHz) 도는
/// 자리라 그 중복이 `bang_bang_swing_planning_does_not_block_physics_step`
/// (스텝 wall-clock 가드)를 부하 중에 넘길 만큼 유의미하다. 실측: 중복 호출
/// 버전은 전체 스위트 병렬 실행에서 그 테스트를 3회 중 2회 실패시켰고, 단일
/// 패스로 합치면 사라진다.
///
/// 반환값의 관절 목표는 `Option` — 레일 x는 순수 기하라 항상 나오지만 IK는
/// 수렴 못 할 수 있고, 그때도 레일 선추종은 계속돼야 한다.
pub fn plan_coarse_track_targets(
    arm: &Arm,
    predictions: &[Prediction],
) -> Option<(f64, Option<Joints>)> {
    let target = coarse_track_geometry(arm, predictions)?;
    let rail = arm.rail.as_ref()?;
    let hint = robot::Pose::new(rail.default_x(), arm.default_joints.clone());
    let joints = arm
        .inverse_pose_with_rail(target.reachable, target.desired_normal, &hint)
        .ok()
        .map(|pose| pose.joints);
    return Some((target.rail_x, joints));
}

/// coarse 추종 목표 기하 — IK 이전 단계까지.
struct CoarseTrackTarget {
    /// 레일 목표 x [m] (`rail.clamp_x` 순수 기하 — IK 불필요).
    rail_x: f64,
    /// 팔 도달 구 안으로 클램프한 라켓 중심.
    reachable: crate::Point3,
    /// 원하는 라켓 면 법선.
    desired_normal: Vector3<f64>,
}

fn coarse_track_geometry(arm: &Arm, predictions: &[Prediction]) -> Option<CoarseTrackTarget> {
    // 예측 hit plane들 중 로봇에 가장 가까운(= 가장 도달 가능성 높은) 하나를
    // 고른다. 가장 먼 평면은 공이 아직 높이 떠 있어 팔 도달권 밖이라, rough
    // 추종엔 base에 제일 가까운 임팩트가 "가장 관련 있는" 목표다. 레일이 x를
    // 담당하므로 거리 비교에서 x는 빼고 y-z 오프셋만 본다(레일로 못 줄이는 축).
    let prediction = predictions
        .iter()
        .filter(|prediction| {
            prediction.time_to_impact_secs.is_finite() && prediction.time_to_impact_secs > 0.0
        })
        .min_by(|left, right| {
            let cost = |prediction: &Prediction| {
                let impact = prediction.impact_position.coords;
                (impact.y - arm.base.coords.y).hypot(impact.z - arm.base.coords.z)
            };
            cost(left)
                .partial_cmp(&cost(right))
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;

    let impact_position = prediction.impact_position;
    let v_in = prediction.incoming_velocity;
    let v_out = Impact::rally_return(impact_position, v_in);
    let delta = v_out - v_in;
    if delta.norm() < 1e-6 {
        return None;
    }
    let desired_normal = delta.normalize();
    let racket_center = crate::Point3::from(
        impact_position.coords
            - desired_normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z),
    );
    // rough 단계라 예측 임팩트가 아직 팔 도달권 밖(공이 높이 떠 있는 초기
    // 비행)이어도, 레일 x라도 미리 맞추도록 도달 구 안으로 클램프한다
    // (y=접수 깊이 우선 보존). coarse 추종은 레일이 있는 로봇 대상.
    let rail = arm.rail.as_ref()?;
    let (rail_x, reachable) = arm.clamp_impact_for_rail(rail, racket_center);
    return Some(CoarseTrackTarget {
        rail_x,
        reachable,
        desired_normal,
    });
}

/// 스윙(혹은 랠리) 뒤 로봇을 중앙 포즈(관절 `default_joints`, 레일 `default_x`
/// = 테이블 폭 중앙)로 되돌리는 궤적을 계획한다.
///
/// 레일의 `home_x`(원점, x=0)는 "대기 위치"일 뿐 테이블 중앙이 아니다 —
/// 여기서 되돌아갈 곳은 `LinearRail::default_x()`(`(x_min+x_max)*0.5`), 즉
/// 테이블 폭 한가운데다. 실제 로봇은 모터 토크 한계 때문에 레일 한쪽
/// 끝에서 반대쪽 끝으로 급하게 움직이는 궤적을 못 만든다 — 매 스윙 뒤 항상
/// 중앙으로 복귀시켜 다음 스윙의 시작 조건을 일정하게 유지한다. 볼 예측이
/// 없으므로 `plan_swing`과 달리 목표 소요 시간이 정해져 있지 않다 — 관절·
/// 레일 속도/가속/토크 한계(`kinematic_limit_violation`·`peak_torque_utilization`)를 만족할 때까지
/// 소요 시간을 점진적으로 늘려가며 찾는다.
pub fn plan_return_to_center(arm: &Arm, start: &robot::Pose) -> Result<Trajectory, DomainError> {
    let center_joints = arm.default_joints.clone();
    let center_rail_x = arm
        .rail
        .as_ref()
        .map(|rail| rail.default_x())
        .unwrap_or(start.rail_x);

    let start_velocity = vec![0.0; start.joints.values.len()];
    let end_velocity = vec![0.0; center_joints.values.len()];

    // 끝속도가 항상 0이라 `fit_end_velocity`의 스케일링은 아무 것도 못 바꾼다
    // (0에 뭘 곱해도 0) — 첫 시도부터 웬만하면 통과하도록, 실제 이동 거리
    // 기준 등속 근사(0.5배 여유, quintic 첨두 속도가 평균보다 크므로)로 시작
    // 시간을 추정해 무의미한 재시도(각 32회 반복)를 줄인다.
    let joint_distance = start
        .joints
        .values
        .iter()
        .zip(center_joints.values.iter())
        .map(|(actual, home)| (actual - home).abs())
        .fold(0.0_f64, f64::max);
    let rail_distance = (start.rail_x - center_rail_x).abs();
    let joint_time_estimate = if arm.max_joint_speed > 0.0 {
        joint_distance / (arm.max_joint_speed * 0.5)
    } else {
        0.0
    };
    let rail_time_estimate = arm.rail.as_ref().map_or(0.0, |rail| {
        if rail.max_speed > 0.0 {
            rail_distance / (rail.max_speed * 0.5)
        } else {
            0.0
        }
    });

    let mut duration = joint_time_estimate
        .max(rail_time_estimate)
        .max(RETURN_TO_CENTER_MIN_SECS);
    let mut last_error = None;
    while duration <= RETURN_TO_CENTER_MAX_SECS {
        let rail = Rail {
            start: start.rail_x,
            end: center_rail_x,
            start_velocity: 0.0,
            end_velocity: 0.0,
        };
        match build_feasible_trajectory(
            arm,
            &start.joints,
            center_joints.clone(),
            start_velocity.clone(),
            end_velocity.clone(),
            duration,
            rail,
        ) {
            Ok(trajectory) => return Ok(trajectory),
            Err(error) => {
                last_error = Some(error);
                duration *= RETURN_TO_CENTER_GROWTH;
            }
        }
    }
    return Err(DomainError::InfeasibleSwing(last_error.unwrap_or(
        SwingPlanError::InverseKinematicsNoSolution {
            target_x: center_rail_x,
            target_y: 0.0,
            target_z: table::SURFACE_Z,
        },
    )));
}

/// 속도/가속 한계 안에 들어오는 quintic을 만든다.
///
/// 종료 위치는 항상 임팩트 IK 해. 끝속도는 한계 안으로 스케일하되
/// 타격 모드에서는 0으로 버리지 않는다 (최소 스케일 유지).
fn build_feasible_trajectory(
    arm: &Arm,
    start: &Joints,
    end: Joints,
    start_velocity: Vec<f64>,
    end_velocity: Vec<f64>,
    duration: f64,
    rail: Rail,
) -> Result<Trajectory, SwingPlanError> {
    let (fitted, fitted_rail) = fit_end_velocity(
        arm,
        start,
        &end,
        &start_velocity,
        end_velocity,
        duration,
        rail,
    );
    let trajectory = trajectory_with_follow_through(
        arm,
        start,
        &end,
        start_velocity,
        fitted,
        duration,
        fitted_rail,
    );
    // 두 원인(관절 각도/속도 vs 토크)을 나눠 보고한다 — 어느 쪽이 병목인지에
    // 따라 대응이 완전히 다르기 때문(전자는 기구학/마운트, 후자는 모터 선정).
    if let Some(violated) = kinematic_limit_violation(arm, &trajectory) {
        return Err(SwingPlanError::TrajectoryExceedsLimits {
            rail_end_x: fitted_rail.end,
            violated,
        });
    }
    let torque_utilization = peak_torque_utilization(arm, &trajectory);
    if torque_utilization > 1.0 {
        return Err(SwingPlanError::TrajectoryExceedsTorque {
            rail_end_x: fitted_rail.end,
            utilization: torque_utilization,
        });
    }
    if !trajectory_collision_free(arm, &trajectory) {
        let depth = {
            let samples = (trajectory.duration_secs / 0.005).ceil() as usize;
            let mut worst = 0.0_f64;
            for index in 0..=samples.max(1) {
                let time = trajectory.duration_secs * index as f64 / samples.max(1) as f64;
                let joints = trajectory.sample_at(time);
                let rail_x = trajectory.sample_rail_at(time);
                worst = worst.max(crate::robot::collision::table_penetration(
                    arm, rail_x, &joints,
                ));
            }
            worst
        };
        return Err(SwingPlanError::TablePenetration {
            target_x: fitted_rail.end,
            target_y: 0.0,
            target_z: table::SURFACE_Z,
            depth,
        });
    }
    return Ok(trajectory);
}

fn trajectory_with_follow_through(
    arm: &Arm,
    start: &Joints,
    impact: &Joints,
    start_velocity: Vec<f64>,
    impact_velocity: Vec<f64>,
    impact_time: f64,
    rail: Rail,
) -> Trajectory {
    let follow_time = defaults::ControlParams::default().swing_follow_through_secs;
    let mut end_values = impact.values.clone();
    for (index, (value, velocity)) in end_values
        .iter_mut()
        .zip(impact_velocity.iter())
        .enumerate()
    {
        *value += velocity * follow_time * 0.5;
        if let Some(limit) = arm.joint_limit(index) {
            *value = (*value).clamp(limit.min, limit.max);
        }
    }
    let follow_rail_x = arm.rail.as_ref().map_or(rail.end, |linear| {
        linear.clamp_x(rail.end + rail.end_velocity * follow_time * 0.5)
    });
    return Trajectory::with_follow_through(
        start.clone(),
        impact.clone(),
        Joints { values: end_values },
        start_velocity,
        impact_velocity,
        vec![0.0; impact.values.len()],
        impact_time,
        impact_time + follow_time,
        rail,
        follow_rail_x,
        0.0,
    );
}

fn trajectory_collision_free(arm: &Arm, trajectory: &Trajectory) -> bool {
    let samples = (trajectory.duration_secs / 0.005).ceil() as usize;
    for index in 0..=samples.max(1) {
        let time = trajectory.duration_secs * index as f64 / samples.max(1) as f64;
        let joints = trajectory.sample_at(time);
        let rail_x = trajectory.sample_rail_at(time);
        if crate::robot::collision::table_penetration(arm, rail_x, &joints) > 1e-3 {
            return false;
        }
    }
    return true;
}

/// 궤적 전 구간을 샘플해 각 관절의 `|토크| / 토크한계` 최악 비율을 구한다.
///
/// Newton-Euler 역동역학으로 관절 토크를 계산하고, per-joint 연속 토크 한계
/// (`Arm::joint_torque_limits`) 대비 이용률을 본다. 반환값 `<= 1.0` 이면 모든
/// 관절이 토크 한계 안. 한계가 무한(`f64::INFINITY`)인 관절은 무시한다.
///
/// 강체 링크 관성뿐 아니라 모터 회전자·기어박스 반사관성
/// (`Arm::joint_reflected_inertias`)까지 포함한
/// `required_joint_torques_with_rotor_into`를 쓴다 — 감속비 200:1이면
/// 반사관성이 링크 관성과 같은 자릿수라, 빼면 여유를 낙관적으로 본다(WP8).
/// 관절 마찰은 아직 미모델이며 `CONTINUOUS_TORQUE_DERATE`가 그 몫을 흡수한다.
fn peak_torque_utilization(arm: &Arm, trajectory: &Trajectory) -> f64 {
    // 토크 한계가 전부 무한(무제한)이면 동역학을 돌릴 필요가 없다.
    if arm
        .joint_torque_limits
        .iter()
        .all(|limit| !limit.is_finite())
    {
        return 0.0;
    }
    // 10ms 간격. quintic 가속 곡선은 매끄러워 이 간격이면 첨두 토크를 <1%
    // 오차로 잡으면서 Newton-Euler 호출 수를 절반으로 줄인다(계획 지연 감소).
    let samples = (trajectory.duration_secs / 0.01).ceil().max(1.0) as usize;
    // 세그먼트를 한 번만 만들고(관절당 3x3 LU) 샘플마다 재사용한다.
    let (pre, post) = trajectory.joint_segments();
    let n = pre.len();
    let mut joints = Joints {
        values: vec![0.0; n],
    };
    let mut velocities = vec![0.0; n];
    let mut accelerations = vec![0.0; n];
    // 스크래치·출력 버퍼를 한 번만 만들어 모든 샘플에서 재사용(힙 할당 회피).
    let mut scratch = crate::robot::dynamics::RneaScratch::new();
    let mut torques = vec![0.0; n];
    let mut worst = 0.0_f64;
    for index in 0..=samples {
        let time = trajectory.duration_secs * index as f64 / samples as f64;
        let (segments, local_t) = if time <= trajectory.impact_time_secs
            || trajectory.duration_secs <= trajectory.impact_time_secs
        {
            (&pre, time)
        } else {
            (&post, time - trajectory.impact_time_secs)
        };
        for i in 0..n {
            let (q, qd, qdd) = segments[i].sample(local_t);
            joints.values[i] = q;
            velocities[i] = qd;
            accelerations[i] = qdd;
        }
        crate::robot::dynamics::required_joint_torques_with_rotor_into(
            arm,
            &joints,
            &velocities,
            &accelerations,
            &mut scratch,
            &mut torques,
        );
        for (torque, &limit) in torques.iter().zip(arm.joint_torque_limits.iter()) {
            if limit.is_finite() && limit > 0.0 {
                worst = worst.max(torque.abs() / limit);
            }
        }
    }
    return worst;
}

/// 토크를 제외한 기구학 한계(관절 속도/가속/각도 범위, 레일 속도/범위)만 본다.
/// 토크 샘플링(Newton-Euler)이 상대적으로 비싸서, 토크 이용률을 이미 따로
/// 계산한 호출부(`fit_end_velocity`)가 중복 계산을 피하도록 분리했다.
fn kinematic_limits_ok(arm: &Arm, trajectory: &Trajectory) -> bool {
    return kinematic_limit_violation(arm, trajectory).is_none();
}

/// 어떤 기구학 한계를 위반했는지 이름을 돌려준다 (`None`이면 위반 없음).
///
/// 단순 bool이면 "궤적이 한계를 넘음"까지만 알 수 있어, 마운트/슈터
/// 튜닝으로 고칠 수 있는 문제인지(관절 각도·레일 범위) 아니면 시간
/// 예산 문제인지(속도·가속) 구분이 안 된다. 실제로 이 구분이 없어서
/// 2026-07-23 조사가 한동안 엉뚱한 축(리치/관절속도 재보정)을 팠다.
fn kinematic_limit_violation(arm: &Arm, trajectory: &Trajectory) -> Option<&'static str> {
    if trajectory.peak_joint_speed() > arm.max_joint_speed {
        return Some("관절 속도");
    }
    if trajectory.peak_joint_acceleration() > defaults::ControlParams::default().max_joint_accel {
        return Some("관절 각가속도");
    }
    if arm
        .rail
        .as_ref()
        .is_some_and(|rail| trajectory.peak_rail_speed() > rail.max_speed)
    {
        return Some("레일 속도");
    }
    // 레일 가속도 검사 — **2026-07-30 임시 비활성화, 켜지 말 것 (재측정 전까지)**.
    //
    // WP5/WP2a: 예전엔 이 항이 없어 레일이 실제로 못 내는 가속을 요구하는
    // 궤적도 "OK"로 통과시켰다(실측: dx=0.25/0.50m 시나리오 다수 행이 τ
    // 통과인데도 실기 레일 가속(당시 12 m/s²)을 최대 4.35배 초과). 검사
    // 로직 자체는 옳다 — 하지만 `RAIL_ACCEL_M_S2 = 12.0`
    // (`defaults/planner.rs`)이 `docs/superpowers/plans/2026-07-22-axl-rail.md`에
    // "board units, tune on bench; values above are placeholders"로 명시된
    // **미검증 placeholder**였다. 이 검사를 켜자 eval+랜덤 67샷 그리드
    // 커밋률이 76%(52/67, Phase 0 기준)→15%(10/67)로 붕괴했는데, 사용자가
    // 실기에서 리니어 모터가 "매우 빠르게" 움직이는 걸 직접 관찰했다고
    // 확인해 줬다 — 즉 12.0 m/s²가 실제보다 훨씬 보수적일 가능성이 높다.
    // 리포지토리 전체를 검색해도 레일 모터 실측 사양은 어디에도 없다(같은
    // 문서가 최초부터 placeholder라고 밝힘). **사용자 결정(2026-07-30)**:
    // 실측 전까지 이 검사는 꺼둔다 — 틀린 값을 강제해 멀쩡한 스윙을
    // 대량으로 거절하는 것이, 몇몇 실제 레일 가속 초과 궤적을 놓치는 것보다
    // 지금 당장은 더 나쁘다. 재측정되면(벤치 스텝 응답 등) `RAIL_ACCEL_M_S2`를
    // 갱신하고 아래 `if false`를 지워 다시 켤 것 — 검사·계측
    // (`Trajectory::peak_rail_acceleration`)은 그대로 남겨둔다.
    if false
        && arm.rail.is_some()
        && trajectory.peak_rail_acceleration() > crate::defaults::motion::RAIL_ACCEL_M_S2
    {
        return Some("레일 가속도");
    }
    let samples = (trajectory.duration_secs / 0.002).ceil() as usize;
    for index in 0..=samples.max(1) {
        let time = trajectory.duration_secs * index as f64 / samples.max(1) as f64;
        if !arm.joints_in_limits(&trajectory.sample_at(time)) {
            return Some("관절 각도 범위");
        }
        if let Some(rail) = &arm.rail {
            let x = trajectory.sample_rail_at(time);
            if !(rail.x_min..=rail.x_max).contains(&x) {
                return Some("레일 이동 범위");
            }
        }
    }
    return None;
}

/// 이분탐색 스텝 수. 배율 해상도 `2^-12 ≈ 0.024%` — 관절속도 한계 대비
/// 무의미할 만큼 촘촘하고, 실현가능 판정 호출 수는 12회로 상한이 고정된다.
const FIT_BISECTION_STEPS: usize = 12;

/// quintic이 관절 속도/각가속도/토크 한계 안에 들어오는 **가장 큰** 임팩트
/// 각속도 배율을 이분탐색한다.
///
/// 예전 구현은 `min(speed_scale, accel_scale, torque_scale) × 0.95`를 최대
/// 32회 반복하는 고정점 반복이었다. WP7 감사(eval 30샷 + 랜덤 5×5 격자,
/// `wp7::diag_downscale_audit_*`)가 측정한 세 가지 문제:
///
/// 1. **매 반복의 0.95 마진이 최종 배율에 그대로 남는다.** 커밋된 스윙의
///    50~67%가 정확히 1회 반복으로 끝나는데, 그 1회가 곱하는 0.95가 곧
///    최종 손실이었다 — 실측 최적 배율은 ~0.9997이라 5%를 그냥 버렸다.
///    (외부 관측: WP2a 스윕에서 달성 `v_r·n`이 정확히 95.0%/100.0% 두
///    값으로만 갈리던 현상이 이것이다.)
/// 2. **`0.95` 추정은 한 번에 맞을 수 없다.** quintic 첨두 속도는 끝속도에
///    비례하지 않는다(Δq가 만드는 위치 구동 성분이 배율과 무관하게 남는다).
///    그래서 `limit/peak`은 필요한 감소량을 항상 과소평가하고 반복이
///    필수였다. 이분탐색은 `low`가 **항상 검증된 실현가능 배율**이라 추정이
///    필요 없고, 한계를 넘는 값을 반환하는 일이 원리적으로 없다.
/// 3. **실현 불가능한 계획에 32회를 전부 쓴다.** 전체 계획 시도의 55~58%가
///    32회 상한까지 돌며 매 반복 Newton-Euler 토크 샘플링을 했는데, 전부
///    "끝속도를 0으로 해도 실현 불가"(위치 이동 자체가 한계 초과)여서
///    단 1회 검사로 판별 가능했다. 이 경로는 최대 1 kHz로 도는 커밋
///    경로이고 이미 wall-clock 가드 테스트가 있다. `plan_return_to_center`의
///    기존 주석("끝속도가 항상 0이라 … 무의미한 재시도(각 32회 반복)")이
///    같은 병리를 반대편에서 이미 기록해 두고 있었다.
///
/// 호출 비용: 이미 실현가능하면 1회(예전과 동일), 어떤 배율로도 불가능하면
/// 2회(예전 32회), 축소가 필요하면 `2 + FIT_BISECTION_STEPS`회.
///
/// 실현가능 판정에 테이블 충돌은 넣지 않는다 — 배율에 대해 단조가 아니고,
/// 최종 검증은 [`build_feasible_trajectory`]가 한다(예전과 동일).
fn fit_end_velocity(
    arm: &Arm,
    start: &Joints,
    end: &Joints,
    start_velocity: &[f64],
    end_velocity: Vec<f64>,
    duration: f64,
    rail: Rail,
) -> (Vec<f64>, Rail) {
    let scaled = |scale: f64| -> (Vec<f64>, Rail) {
        let mut scaled_rail = rail;
        scaled_rail.end_velocity = rail.end_velocity * scale;
        return (
            end_velocity.iter().map(|value| value * scale).collect(),
            scaled_rail,
        );
    };
    let feasible = |velocities: Vec<f64>, candidate_rail: Rail| -> bool {
        let trajectory = trajectory_with_follow_through(
            arm,
            start,
            end,
            start_velocity.to_vec(),
            velocities,
            duration,
            candidate_rail,
        );
        return peak_torque_utilization(arm, &trajectory) <= 1.0
            && kinematic_limits_ok(arm, &trajectory);
    };

    if feasible(end_velocity.clone(), rail) {
        return (end_velocity, rail);
    }

    // 끝속도를 0으로 만들어도 실현 불가면 어떤 배율도 통하지 않는다 — 위치
    // 이동 자체(Δq, 레일 이동거리)가 한계를 넘는 경우다. 특히 레일 첨두
    // 속도는 **이동거리/소요시간**이 결정하므로 끝속도 축소로는 줄지 않는다
    // (감사 실측: 레일 속도로 막힌 39~41%는 실현가능 배율이 아예 없었다).
    // 끝속도는 원본을 유지해 호출부의 한계 위반 진단이 "무엇이 실제로
    // 과했는지"를 그대로 보고하게 한다 (타격 의도 유지 — 0으로 버리지 않는다).
    let (zero_velocity, zero_rail) = scaled(0.0);
    if !feasible(zero_velocity.clone(), zero_rail) {
        return (end_velocity, rail);
    }

    // 실현가능 배율이 존재한다 — 가장 큰 것을 이분탐색한다. `low`는 항상
    // 검증된 실현가능 배율이므로 반환값은 반드시 한계 안이다.
    let mut low = 0.0_f64;
    let mut high = 1.0_f64;
    let mut best = zero_velocity;
    let mut best_rail = zero_rail;
    for _ in 0..FIT_BISECTION_STEPS {
        let mid = (low + high) * 0.5;
        let (candidate, candidate_rail) = scaled(mid);
        if feasible(candidate.clone(), candidate_rail) {
            low = mid;
            best = candidate;
            best_rail = candidate_rail;
        } else {
            high = mid;
        }
    }
    return (best, best_rail);
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::constants::table;
    use crate::estimator::Prediction;
    use crate::robot::Arm;

    fn sample_three_dof_arm() -> Arm {
        // 피처 브랜치가 실기 관절속도(~2.88 rad/s)로 검증한 마운트
        // (BASE_Y=-0.02, height=0.05). main의 rail_frame(0.20/0.20)은
        // tools/shot_tune 재튜닝 전까지 시뮬 통합 테스트에서만 쓴다.
        let mount_z = table::SURFACE_Z + 0.05;
        return (*crate::defaults::primitive_4dof_with_mount(-0.02, mount_z)
            .expect("테스트용 4DOF arm")
            .arm)
            .clone();
    }

    fn sample_start(arm: &Arm) -> robot::Pose {
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        return robot::Pose::new(rail_x, arm.default_joints.clone());
    }

    /// 대표 임팩트 높이 [m] — 탁구대 면 위. 1차 조사가 찾아낸 "실현 가능
    /// 대역"(10~30cm)의 한가운데.
    const SAMPLE_IMPACT_HEIGHT_M: f64 = 0.18;

    /// 이 팔이 실제로 마주치는 대표 임팩트 예측.
    ///
    /// 예전에는 "휴지 자세의 FK 위치"를 그대로 임팩트로 썼다 — 휴지 자세가
    /// 관절 한계 중점이던 시절엔 그게 자명하게 도달 가능한 점이라 편했다.
    /// 휴지 자세를 임팩트 자세들 쪽으로 옮긴 뒤(`READY_JOINTS_4DOF`)로는
    /// 그 점이 오히려 **특이점 근처**가 됐다(실측: 관절 2가 9.77 rad/s 요구).
    /// "휴지 자세가 가리키는 곳"은 애초에 물리적 의미가 없는 임팩트라,
    /// 실제 접수 창·실현가능 높이 대역 안의 점으로 바꾼다.
    fn sample_prediction(time_to_impact_secs: f64) -> Prediction {
        return Prediction {
            time_to_impact_secs,
            impact_position: crate::Point3::new(
                table::WIDTH_X * 0.5,
                // main DEFAULT_HIT_PLANE_Y=0.08은 철제 마운트 기준 접수 창 하한.
                // 피처가 검증한 대표 임팩트는 y≈0.18(실현가능 대역 중앙).
                0.18,
                table::SURFACE_Z + SAMPLE_IMPACT_HEIGHT_M,
            ),
            // 튜닝된 슈터가 바운스 뒤 실제로 만드는 조성(수평 ~7 m/s에
            // 완만한 상승)에 맞춘다.
            incoming_velocity: Vector3::new(0.0, -7.0, 0.7),
        };
    }

    #[test]
    fn in_swing_commit_window_bounds() {
        assert!(!in_swing_commit_window(0.05));
        assert!(in_swing_commit_window(
            defaults::ControlParams::default().min_swing_secs + 0.05
        ));
        assert!(in_swing_commit_window(
            defaults::ControlParams::default().swing_commit_max_secs
        ));
        assert!(!in_swing_commit_window(
            defaults::ControlParams::default().swing_commit_max_secs + 0.01
        ));
    }

    #[test]
    fn midcourt_gate_matches_fraction() {
        let limit =
            table::LENGTH_Y * defaults::ControlParams::default().swing_commit_max_ball_y_frac;
        assert!(!ball_past_midcourt_for_commit(limit + 0.01));
        assert!(ball_past_midcourt_for_commit(limit));
        assert!(ball_past_midcourt_for_commit(0.3));
    }

    #[test]
    #[ignore = "realistic joint speed + main rail_frame/hit-plane need shot_tune retune; see .omc/research/known-regressions-realistic-joint-speed.md"]
    fn plan_swing_reaches_impact_with_end_velocity() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let prediction = sample_prediction(0.45);
        let trajectory = plan_swing(&arm, prediction, &start).expect("스윙 계획");
        assert!(trajectory.duration_secs > trajectory.impact_time_secs);
        assert!(
            trajectory
                .end_joints()
                .values
                .iter()
                .zip(trajectory.impact_joints().values.iter())
                .any(|(end, impact)| (end - impact).abs() > 1e-4),
            "임팩트 뒤 팔로스루 관절 이동이 있어야 함"
        );
        let pose = arm
            .forward_kinematics_with_rail(trajectory.rail.end, trajectory.goal_joints())
            .expect("FK");
        let contact = pose.position.coords
            + pose.normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z);
        let desired_normal =
            (Impact::rally_return(prediction.impact_position, prediction.incoming_velocity)
                - prediction.incoming_velocity)
                .normalize();
        assert!((contact.x - prediction.impact_position.coords.x).abs() < 2e-3);
        assert!((contact.y - prediction.impact_position.coords.y).abs() < 2e-3);
        assert!(
            contact.z + 2e-3 >= prediction.impact_position.coords.z,
            "테이블 클램프로 z만 올라갈 수 있음"
        );
        assert!((pose.normal - desired_normal).norm() < 2e-3);
        let dt = 1e-5;
        let before = arm
            .forward_kinematics_with_rail(
                trajectory.sample_rail_at(trajectory.impact_time_secs - dt),
                &trajectory.sample_at(trajectory.impact_time_secs - dt),
            )
            .expect("impact 직전 FK");
        let actual_racket_velocity = (pose.position.coords - before.position.coords) / dt;
        let desired_racket_velocity = Impact::required_racket_velocity(
            prediction.incoming_velocity,
            Impact::rally_return(prediction.impact_position, prediction.incoming_velocity),
            pose.normal,
            defaults::ImpactParams::default().racket_effective_restitution,
        )
        .expect("required racket velocity");
        // 이 샷은 실제 per-joint 토크 한계(derated MX stall) 아래에서는 완전한
        // 목표 라켓 속도를 못 낸다 — 작은 MX-28(elbow/wrist) 모터엔 과한 가속
        // 이라 스윙이 토크로 스로틀된다. 예전 flat 토크 모델
        // (MAX_JOINT_TORQUE=20, 사실상 가속 한계와 동일)에선 정확 일치를
        // 통과했지만, Newton-Euler 동역학에선 물리적으로 제한된다. 따라서
        // "정확히 목표 속도"가 아니라 (1) 목표 방향으로 밀고, (2) 목표를 넘지
        // 않으며, (3) 궤적이 토크 한계에 걸려 있음을 검증한다.
        //
        // 관절 속도 상한도 `Arm::competition()`이 `16.0`(근거 없는 리터럴) 대신
        // 실기 Dynamixel 스펙 기반 `DYNAMIXEL_MAX_JOINT_SPEED_RAD_S`(~2.88 rad/s,
        // `.omc/research/dynamixel-specs.md`)를 쓰도록 바뀌면서 이 시나리오는
        // 토크뿐 아니라 관절 속도로도 스로틀된다 — 두 제약이 겹쳐 `along`이
        // 이전보다 더 낮아진다(관측값 ≈0.173). 임계값을 그만큼 낮춘다: 여전히
        // "유의미하게 목표 방향으로 밀되 넘지 않음"을 검증하되, 이제는 더 느린
        // 실기 팔의 실제 도달 가능 범위를 반영한다.
        let along = actual_racket_velocity.dot(&desired_racket_velocity)
            / desired_racket_velocity.norm_squared();
        assert!(
            along > 0.15 && along < 1.05,
            "라켓 속도가 목표 방향의 유의미한(넘지 않는) 비율이어야: along={along}, \
             actual={actual_racket_velocity:?}, desired={desired_racket_velocity:?}, \
             joint_speed={}, joint_accel={}, rail_speed={}",
            trajectory.peak_joint_speed(),
            trajectory.peak_joint_acceleration(),
            trajectory.peak_rail_speed(),
        );
        let torque_util = peak_torque_utilization(&arm, &trajectory);
        assert!(
            torque_util <= 1.0 + 1e-3,
            "실현 궤적은 토크 한계 안이어야: util={torque_util}"
        );
        assert!(
            torque_util > 0.5,
            "스윙이 토크로 제한됐어야(한계 근처): util={torque_util}"
        );
        assert!(
            crate::robot::collision::table_penetration(
                &arm,
                trajectory.rail.end,
                trajectory.goal_joints()
            ) < 1e-3
        );
        assert!(
            trajectory.end_velocity.iter().any(|v| v.abs() > 0.05),
            "로프트 타격 끝속도가 살아 있어야 함: {:?}",
            trajectory.end_velocity
        );
        assert!(trajectory.peak_joint_speed() <= arm.max_joint_speed * 1.05);
    }

    #[test]
    #[ignore = "realistic joint speed + main rail_frame/hit-plane need shot_tune retune; see .omc/research/known-regressions-realistic-joint-speed.md"]
    fn plan_swing_moves_rail_to_impact_x() {
        let arm = sample_three_dof_arm();
        let start = robot::Pose::new(0.1, arm.default_joints.clone());
        // 레일 목표를 0.8 → 0.5 배로 낮췄다: 5.0 m/s 실기 레일 속도로 재보정한
        // 뒤(이전 12.0 m/s 근거 없는 리터럴), 0.1→1.22m(0.8배)를 0.3초 안에 도는
        // 건 진짜로 실현 불가능해졌다(quintic peak 속도가 5.0 m/s 한계를 넘음).
        // 0.5배는 같은 "레일이 임팩트 x로 움직인다"는 의도를 유지하면서 실제
        // 도달 가능한 거리로 남겨둔다.
        // 임팩트는 레일 중앙(x) × 접수 평면(y) × 실현가능 높이 대역(z).
        // 시작 레일이 0.1이므로 레일이 실제로 x 쪽으로 움직여야 한다.
        let impact = crate::Point3::new(
            table::WIDTH_X * 0.5,
            0.18,
            table::SURFACE_Z + SAMPLE_IMPACT_HEIGHT_M,
        );
        let prediction = Prediction {
            time_to_impact_secs: 0.3,
            impact_position: impact,
            incoming_velocity: Vector3::new(0.0, -7.0, 0.7),
        };
        let trajectory = plan_swing(&arm, prediction, &start).expect("스윙 계획");
        let pose = arm
            .forward_kinematics_with_rail(trajectory.rail.end, trajectory.goal_joints())
            .expect("FK");
        let contact = pose.position.coords
            + pose.normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z);
        assert!((contact.x - impact.coords.x).abs() < 2e-3);
        assert!((trajectory.rail.start - 0.1).abs() < 1e-6);
    }

    #[test]
    #[ignore = "realistic joint speed + main rail_frame/hit-plane need shot_tune retune; see .omc/research/known-regressions-realistic-joint-speed.md"]
    fn best_swing_rejects_clamped_contact_and_selects_reachable_candidate() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        // 0.18s는 이 팔이 휴지 자세에서 임팩트 자세까지 quintic으로 가기엔
        // 너무 짧다(실측 최소 ~0.29s) — 이 테스트가 보려는 건 "도달 불가
        // 후보를 버리고 도달 가능 후보를 고르는가"지 시간 예산이 아니라서,
        // 실제로 실현 가능한 시간을 준다.
        let reachable = sample_prediction(0.32);
        let mut unreachable = reachable;
        unreachable.impact_position.coords.x = 100.0;
        unreachable.impact_position.coords.y = 0.55;

        let selected =
            plan_best_swing(&arm, &[unreachable, reachable], &start).expect("reachable candidate");
        assert_eq!(selected.prediction, reachable);
    }

    #[test]
    fn plan_swing_fails_when_insufficient_time() {
        let arm = sample_three_dof_arm();
        let err = plan_swing(&arm, sample_prediction(0.05), &sample_start(&arm)).unwrap_err();
        let DomainError::InfeasibleSwing(SwingPlanError::InsufficientTime {
            time_to_impact_secs,
            min_swing_secs,
        }) = err
        else {
            panic!("InsufficientTime 기대");
        };
        assert!((time_to_impact_secs - 0.05).abs() < f64::EPSILON);
        assert!(
            (min_swing_secs - defaults::ControlParams::default().min_swing_secs).abs()
                < f64::EPSILON
        );
    }

    #[test]
    fn competition_geometry_reachable_with_rail() {
        let arm = crate::defaults::primitive_4dof()
            .expect("competition arm")
            .arm;

        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let far_impact = arm
            .forward_kinematics_with_rail(rail_x, &arm.default_joints)
            .expect("FK")
            .position;
        let start = robot::Pose::new(rail_x, arm.default_joints.clone());
        let prediction = Prediction {
            time_to_impact_secs: 0.22,
            impact_position: far_impact,
            incoming_velocity: Vector3::new(0.0, -7.5, -0.3),
        };
        let trajectory = plan_swing(&arm, prediction, &start).expect("슈터->로봇 기본 샷");
        let pose = arm
            .forward_kinematics_with_rail(trajectory.rail.end, trajectory.goal_joints())
            .expect("impact FK");
        let contact = pose.position.coords
            + pose.normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z);
        assert!((contact.x - far_impact.coords.x).abs() < 2e-3);
        assert!(trajectory.peak_joint_speed() <= arm.max_joint_speed);
        assert_ne!(
            trajectory.goal_joints().values,
            arm.default_joints.values,
            "접수 방향으로 관절 목표가 달라져야 함"
        );
    }

    #[test]
    fn urdf_arm_torque_gate_rejects_or_stays_feasible() {
        // RNEA 하드 게이트: 성공하면 peak τ/limit ≤ 1, 아니면 JointOrTorqueLimit.
        let arm = (*crate::defaults::urdf_4dof().expect("urdf").arm).clone();
        assert!(!arm.aggregated_inertials.is_empty());
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let impact = arm
            .forward_kinematics_with_rail(rail_x, &arm.default_joints)
            .expect("FK")
            .position;
        let start = robot::Pose::new(rail_x, arm.default_joints.clone());
        let prediction = Prediction {
            time_to_impact_secs: 0.22,
            impact_position: impact,
            incoming_velocity: Vector3::new(0.0, -7.5, -0.3),
        };
        match plan_swing(&arm, prediction, &start) {
            Ok(trajectory) => {
                let util = peak_torque_utilization(&arm, &trajectory);
                assert!(
                    util <= 1.0 + 1e-3,
                    "커밋된 궤적 peak τ 이용률이 한계 안이어야 함: util={util}"
                );
            }
            Err(DomainError::InfeasibleSwing(SwingPlanError::JointOrTorqueLimit { .. }))
            | Err(DomainError::InfeasibleSwing(SwingPlanError::TrajectoryExceedsTorque {
                ..
            }))
            | Err(DomainError::InfeasibleSwing(SwingPlanError::NearSingularity { .. }))
            | Err(DomainError::InfeasibleSwing(SwingPlanError::TrajectoryExceedsLimits {
                ..
            })) => {}
            Err(other) => panic!("토크/한계 게이트 또는 성공만 기대, got {other}"),
        }
    }

    /// WP8 계측 — 회전자 반사관성 항 추가 전/후 `peak_torque_utilization` 비교.
    ///
    /// 같은 궤적을 두 모델(강체만 / 강체+반사관성)로 평가해 모델 변경만의
    /// 효과를 분리하고, 이어서 각 모델로 **재계획**했을 때 달성 라켓 속도가
    /// 어떻게 달라지는지(다운스케일의 실질 비용)까지 본다.
    ///
    /// ```text
    /// cargo test -p pingpong-bot --lib diag_reflected_inertia -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "진단용 계측 — 수치를 stdout으로 뽑는다"]
    fn diag_reflected_inertia_torque_utilization() {
        /// 관절별 `max |τ| / limit` — `peak_torque_utilization`의 per-joint 버전.
        fn per_joint_utilization(arm: &Arm, trajectory: &Trajectory) -> Vec<f64> {
            let n = arm.joint_count();
            let samples = (trajectory.duration_secs / 0.005).ceil().max(1.0) as usize;
            let mut scratch = crate::robot::dynamics::RneaScratch::new();
            let mut torques = vec![0.0; n];
            let mut worst = vec![0.0_f64; n];
            for index in 0..=samples {
                let time = trajectory.duration_secs * index as f64 / samples as f64;
                let joints = trajectory.sample_at(time);
                let qd = trajectory.sample_velocity_at(time);
                let qdd = trajectory.sample_acceleration_at(time);
                crate::robot::dynamics::required_joint_torques_with_rotor_into(
                    arm,
                    &joints,
                    &qd,
                    &qdd,
                    &mut scratch,
                    &mut torques,
                );
                for i in 0..n {
                    worst[i] = worst[i].max(torques[i].abs() / arm.joint_torque_limits[i]);
                }
            }
            return worst;
        }

        fn fmt(values: &[f64]) -> String {
            return values
                .iter()
                .map(|v| format!("{v:.3}"))
                .collect::<Vec<_>>()
                .join(" ");
        }

        let urdf_arm = (*crate::defaults::urdf_4dof().expect("urdf").arm).clone();
        let urdf_rail_x = urdf_arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let urdf_impact = urdf_arm
            .forward_kinematics_with_rail(urdf_rail_x, &urdf_arm.default_joints)
            .expect("FK")
            .position;

        let scenarios: Vec<(&str, Arm, Vec<(f64, Prediction)>)> = vec![
            (
                "primitive_4dof / 대표 임팩트 (y=0.18, z=table+0.18)",
                sample_three_dof_arm(),
                [0.22_f64, 0.25, 0.28, 0.30, 0.35, 0.40]
                    .into_iter()
                    .map(|tti| (tti, sample_prediction(tti)))
                    .collect(),
            ),
            (
                "urdf_4dof / 휴지 자세 FK 임팩트 (기존 토크게이트 테스트와 동일)",
                urdf_arm,
                [0.22_f64, 0.25, 0.30, 0.35]
                    .into_iter()
                    .map(|tti| {
                        (
                            tti,
                            Prediction {
                                time_to_impact_secs: tti,
                                impact_position: urdf_impact,
                                incoming_velocity: Vector3::new(0.0, -7.5, -0.3),
                            },
                        )
                    })
                    .collect(),
            ),
        ];

        for (label, arm, cases) in scenarios {
            let n = arm.joint_count();
            let rigid_only = arm.clone().with_joint_reflected_inertias(vec![0.0; n]);
            eprintln!("\n=== {label} ===");
            eprintln!("  반사관성 [kg·m²] = {}", fmt(&arm.joint_reflected_inertias));
            eprintln!("  토크한계  [N·m]  = {}", fmt(&arm.joint_torque_limits));

            for (tti, prediction) in cases {
                let start = sample_start(&arm);
                // 반사관성 없이 계획한 궤적을 **고정**하고 두 모델로 평가한다 —
                // 모델 변경만의 효과를 다운스케일 반응과 분리하기 위해서.
                let Ok(trajectory) = plan_swing(&rigid_only, prediction, &start) else {
                    eprintln!("  tti={tti:.2}s: 강체 모델로도 계획 실패 — 건너뜀");
                    continue;
                };
                let before = peak_torque_utilization(&rigid_only, &trajectory);
                let after = peak_torque_utilization(&arm, &trajectory);
                eprintln!(
                    "  tti={tti:.2}s peak q̈={:7.1} rad/s² | util 전={before:.3} 후={after:.3} ({:.2}×)",
                    trajectory.peak_joint_acceleration(),
                    after / before.max(1e-9),
                );
                eprintln!(
                    "            관절별 util 전=[{}] 후=[{}]",
                    fmt(&per_joint_utilization(&rigid_only, &trajectory)),
                    fmt(&per_joint_utilization(&arm, &trajectory)),
                );
                // 반사관성을 켠 채 재계획하면 게이트가 끝속도를 깎을 수 있다.
                match plan_swing(&arm, prediction, &start) {
                    Ok(replanned) => eprintln!(
                        "            재계획 성공: util={:.3}, peak q̇ 비={:.3}",
                        peak_torque_utilization(&arm, &replanned),
                        replanned.peak_joint_speed() / trajectory.peak_joint_speed().max(1e-9),
                    ),
                    Err(e) => eprintln!("            재계획 실패: {e}"),
                }
            }
        }
    }

    #[test]
    fn trajectory_limits_reject_internal_joint_overshoot() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let limit = arm.joint_limit(1).expect("bounded shoulder");
        let mut impact = start.joints.clone();
        impact.values[1] = limit.max;
        let mut impact_velocity = vec![0.0; impact.values.len()];
        impact_velocity[1] = 4.0;
        let trajectory = trajectory_with_follow_through(
            &arm,
            &start.joints,
            &impact,
            vec![0.0; impact.values.len()],
            impact_velocity,
            0.30,
            Rail::fixed(start.rail_x),
        );
        // 한계 위반은 "무엇을" 어겼는지까지 잡힌다 (이전엔 bool 하나였다).
        assert_eq!(
            kinematic_limit_violation(&arm, &trajectory),
            Some("관절 속도"),
            "임팩트 각속도 4.0 rad/s는 한계 {:.2} rad/s를 넘어야 함",
            arm.max_joint_speed
        );
    }

    // ---------------------------------------------------------------------
    // WP2a — 커밋 시간창(`min_swing_secs`/`swing_commit_max_secs`) 검증
    // ---------------------------------------------------------------------

    /// `plan_swing`과 동일하지만 `min_swing_secs` 게이트를 건너뛴다.
    ///
    /// WP2a의 질문은 "0.08 s가 **실측** 하한이냐, 옛 추정치냐"다. `plan_swing`은
    /// 그 값 아래를 궤적 생성 전에 즉시 거절하므로, 게이트를 통과시켰을 때
    /// 물리(관절속도/가속/토크)가 실제로 무엇을 말하는지 볼 수 없다. 이
    /// 헬퍼는 게이트만 빼고 나머지는 `plan_swing`과 동일하게 통과시킨다.
    fn plan_swing_without_time_gate(
        arm: &Arm,
        prediction: Prediction,
        start: &robot::Pose,
    ) -> Result<Trajectory, DomainError> {
        let target = solve_impact_target(arm, &prediction, start)?;
        let start_velocity = vec![0.0; start.joints.values.len()];
        let rail_motion = Rail {
            start: start.rail_x,
            end: target.pose.rail_x,
            start_velocity: 0.0,
            end_velocity: target.rail_velocity,
        };
        return build_feasible_trajectory(
            arm,
            &start.joints,
            target.pose.joints,
            start_velocity,
            target.joint_velocities,
            prediction.time_to_impact_secs,
            rail_motion,
        )
        .map_err(DomainError::InfeasibleSwing);
    }

    /// [`sample_prediction`]과 같지만 임팩트 x를 옮긴다 — **레일이 실제로
    /// 이동해야 하는** 시나리오를 만든다.
    ///
    /// 왜 필요한가: `sample_prediction`의 임팩트 x는 `WIDTH_X*0.5`이고 시작
    /// 레일 x도 `rail.default_x()`(같은 중앙)라 레일 이동량이 **정확히 0**이다.
    /// 그 fixture로는 레일 속도·가속 컬럼이 항상 0으로 나와 레일 관련 결론을
    /// 아무것도 못 낸다(WP2a 1차 시도에서 실제로 그렇게 나왔다).
    fn sample_prediction_at_dx(time_to_impact_secs: f64, dx: f64) -> Prediction {
        let mut prediction = sample_prediction(time_to_impact_secs);
        prediction.impact_position = crate::Point3::new(
            prediction.impact_position.coords.x + dx,
            prediction.impact_position.coords.y,
            prediction.impact_position.coords.z,
        );
        return prediction;
    }

    /// 유한차분 순방향 자코비안으로 라켓 속도를 추정한다
    /// (`tools/swing_bench`의 `racket_velocity_estimate`와 동일한 방식).
    fn racket_velocity_fd(
        arm: &Arm,
        rail_x: f64,
        rail_velocity: f64,
        joints: &Joints,
        joint_velocities: &[f64],
    ) -> Option<Vector3<f64>> {
        const STEP: f64 = 1e-6;
        let base = arm.forward_kinematics_with_rail(rail_x, joints)?;
        let nudged: Vec<f64> = joints
            .values
            .iter()
            .zip(joint_velocities)
            .map(|(q, v)| q + v * STEP)
            .collect();
        let perturbed = arm.forward_kinematics_with_rail(
            rail_x + rail_velocity * STEP,
            &Joints::from_slice(&nudged),
        )?;
        return Some((perturbed.position.coords - base.position.coords) / STEP);
    }

    /// 짧은 실패 사유 라벨 — 표 한 칸에 들어가게 줄인다.
    fn failure_label(error: &DomainError) -> String {
        let DomainError::InfeasibleSwing(err) = error else {
            return "기타".to_string();
        };
        return match err {
            SwingPlanError::InsufficientTime { .. } => "시간부족(게이트)".to_string(),
            SwingPlanError::InverseKinematicsNoSolution { .. } => "IK 해없음".to_string(),
            SwingPlanError::ReturnVelocityUnreachable { .. } => "리턴속도불가".to_string(),
            SwingPlanError::NearSingularity { joint_index, .. } => {
                format!("특이점근접(q{joint_index})")
            }
            SwingPlanError::TrajectoryExceedsLimits { violated, .. } => {
                format!("한계:{violated}")
            }
            SwingPlanError::TrajectoryExceedsTorque { utilization, .. } => {
                format!("토크 {utilization:.2}x")
            }
            _ => "기타".to_string(),
        };
    }

    /// [진단] WP2a — time-to-impact 스윕으로 커밋 시간창 경계를 실측 검증한다.
    ///
    /// 대표 임팩트 목표를 고정하고 time-to-impact만 0.03~0.60 s로 훑어, 각
    /// 값에서 quintic 경로가 실제로 실행 가능한지와 스윙 품질(달성 `v_r·n`,
    /// 토크 여유, 관절/레일 이용률)을 기록한다. `min_swing_secs` 게이트를 **끈
    /// 상태**로 계획해 "게이트가 막고 있을 뿐 물리적으로는 가능한" 구간이
    /// 있는지 본다 — 그게 WP2a의 핵심 질문이다.
    ///
    /// 레일 이동량이 다른 3개 시나리오(dx = 0 / 0.25 / 0.50 m)를 함께 돈다.
    /// `rail_a` 컬럼이 1.0을 넘는 행은 "플래너는 통과시키지만 실기 레일은 못
    /// 내는" 궤적이다 — `kinematic_limit_violation`이 레일 속도만 검사하기
    /// 때문(WP5 발견, 이번 실험의 범위 추가 항목).
    #[test]
    #[ignore = "순수 진단(스윕). 실행: cargo test --lib diag_commit_window_feasibility_sweep \
                -- --ignored --nocapture"]
    fn diag_commit_window_feasibility_sweep() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let control = defaults::ControlParams::default();
        let rail_max_speed = arm.rail.as_ref().map_or(f64::INFINITY, |r| r.max_speed);
        let rail_accel_limit = crate::defaults::motion::RAIL_ACCEL_M_S2;

        println!(
            "arm.max_joint_speed={:.3} rad/s, max_joint_accel={:.0} rad/s², \
             rail.max_speed={:.2} m/s, RAIL_ACCEL_M_S2={:.1} m/s²",
            arm.max_joint_speed, control.max_joint_accel, rail_max_speed, rail_accel_limit
        );
        println!(
            "게이트: min_swing_secs={:.3}s, swing_commit_max_secs={:.3}s (게이트 안=\"안\")",
            control.min_swing_secs, control.swing_commit_max_secs
        );

        for dx in [0.0_f64, 0.25, 0.50] {
            println!();
            println!("===== 임팩트 x 오프셋 dx={dx:+.2} m (레일 이동량) =====");
            println!(
                "{:>6} {:>5} {:>5} {:>8} {:>8} {:>7} {:>7} {:>7} {:>7} {:>7}  {}",
                "tti", "게이트", "계획", "req|vr|", "got|vr|", "vr·n%", "q̇/lim", "q̈/lim",
                "rail_v", "rail_a", "사유",
            );

            let mut tti_milli = 30_u32;
            while tti_milli <= 600 {
                let tti = f64::from(tti_milli) / 1000.0;
                let prediction = sample_prediction_at_dx(tti, dx);
                let in_gate = tti >= control.min_swing_secs && in_swing_commit_window(tti);
                let gate_mark = if in_gate { "안" } else { "밖" };
                let required = solve_impact_target(&arm, &prediction, &start)
                    .ok()
                    .map(|t| t.racket_velocity);

                match plan_swing_without_time_gate(&arm, prediction, &start) {
                    Ok(trajectory) => {
                        let impact_joints = trajectory.impact_joints().clone();
                        let achieved = racket_velocity_fd(
                            &arm,
                            trajectory.rail.end,
                            trajectory.rail.end_velocity,
                            &impact_joints,
                            &trajectory.end_velocity,
                        );
                        let normal = arm
                            .forward_kinematics_with_rail(trajectory.rail.end, &impact_joints)
                            .map(|pose| pose.normal);
                        let ratio_pct = match (required, achieved, normal) {
                            (Some(req), Some(got), Some(n)) => {
                                let req_n = req.dot(&n).abs();
                                if req_n > 1e-9 {
                                    got.dot(&n).abs() / req_n * 100.0
                                } else {
                                    f64::NAN
                                }
                            }
                            _ => f64::NAN,
                        };
                        let rail_a_ratio =
                            trajectory.peak_rail_acceleration() / rail_accel_limit;
                        let flag = if rail_a_ratio > 1.0 { " ⚠레일가속초과" } else { "" };
                        println!(
                            "{:>6.3} {:>5} {:>5} {:>8.3} {:>8.3} {:>6.1}% {:>7.2} {:>7.2} \
                             {:>7.2} {:>7.2}  τ={:.2}x{}",
                            tti,
                            gate_mark,
                            "OK",
                            required.map_or(f64::NAN, |v| v.norm()),
                            achieved.map_or(f64::NAN, |v| v.norm()),
                            ratio_pct,
                            trajectory.peak_joint_speed() / arm.max_joint_speed,
                            trajectory.peak_joint_acceleration() / control.max_joint_accel,
                            trajectory.peak_rail_speed() / rail_max_speed,
                            rail_a_ratio,
                            peak_torque_utilization(&arm, &trajectory),
                            flag,
                        );
                    }
                    Err(error) => {
                        println!(
                            "{:>6.3} {:>5} {:>5} {:>8.3} {:>8} {:>7} {:>7} {:>7} {:>7} {:>7}  {}",
                            tti,
                            gate_mark,
                            "FAIL",
                            required.map_or(f64::NAN, |v| v.norm()),
                            "—",
                            "—",
                            "—",
                            "—",
                            "—",
                            "—",
                            failure_label(&error),
                        );
                    }
                }
                // 0.03~0.20은 5ms, 그 위는 20ms — 경계 근처를 촘촘히 본다.
                tti_milli += if tti_milli < 200 { 5 } else { 20 };
            }
        }
    }

    // -----------------------------------------------------------------
    // 백스윙(windup) 휴지 자세 탐색
    // -----------------------------------------------------------------

    /// [`tools/shot_tune`]의 `rest_pose_scenarios`와 같은 격자 — 팔이 실제
    /// 마주치는 대표 임팩트 165개. `solve_impact_target`이 crate 내부 전용
    /// (`pub(crate)`)이라 여기 재구현한다.
    fn windup_rest_pose_scenarios() -> Vec<Prediction> {
        let mut out = Vec::new();
        for &x_frac in &[0.1, 0.3, 0.5, 0.7, 0.9] {
            for &y in &[0.20, 0.30, 0.40, 0.55] {
                for &z_off in &[0.10, 0.18, 0.26, 0.30] {
                    for &(speed, descend) in &[(6.0, -0.15), (7.5, 0.10), (6.5, 0.30)] {
                        out.push(Prediction {
                            time_to_impact_secs: 0.15,
                            impact_position: crate::Point3::new(
                                table::WIDTH_X * x_frac,
                                y,
                                table::SURFACE_Z + z_off,
                            ),
                            incoming_velocity: Vector3::new(0.0, -speed, speed * descend),
                        });
                    }
                }
            }
        }
        return out;
    }

    /// 관절별 Chebyshev 중심 — `rest_pose_search`와 동일한 최소최대 로직.
    fn chebyshev_center(lo: &[f64], hi: &[f64], arm: &Arm) -> Vec<f64> {
        return (0..lo.len())
            .map(|j| {
                let mid = (lo[j] + hi[j]) * 0.5;
                return match arm.joint_limit(j) {
                    Some(limit) => mid.clamp(limit.min, limit.max),
                    None => mid,
                };
            })
            .collect();
    }

    fn worst_dq(candidate: &[f64], samples: &[Vec<f64>]) -> f64 {
        return samples
            .iter()
            .flat_map(|sample| {
                sample
                    .iter()
                    .zip(candidate)
                    .map(|(s, c)| (s - c).abs())
            })
            .fold(0.0_f64, f64::max);
    }

    /// [진단] 백스윙(windup) 휴지 자세 탐색.
    ///
    /// 사용자 관찰: 실제 GUI에서 매 스윙마다 "라켓이 뒤로 당겨지는" 동작이
    /// 반복된다 — `plan_return_to_center`가 팔로스루 뒤 팔을
    /// `READY_JOINTS_4DOF`(임팩트 자세들의 Chebyshev 중심, 즉 **중립** 자세)로
    /// 되돌리는데, 팔로스루는 임팩트 속도 방향으로 더 나아간 상태라
    /// 되돌아가는 동작 자체가 매번 "당겨치는 것과 반대 방향" 회전으로
    /// 보인다. 실제 선수는 대기 자세를 중립이 아니라 **이미 당겨진(backswing)**
    /// 상태로 잡는다 — 그러면 복귀 동작 자체가 다음 스윙의 예비 동작이 된다.
    ///
    /// 방법: 각 대표 임팩트에서 `solve_impact_target`이 실제로 내는 임팩트
    /// 관절각 `q_impact`와 명령 관절속도 `q̇_impact`(NEAR_SINGULARITY 다운스케일
    /// 이후 값 — 실제 커밋되는 값)를 구해, `q_windup = q_impact − q̇_impact · T_w`
    /// 로 시간 `T_w`만큼 되감은 "당겨진" 자세를 만든다. 여러 `T_w`에 대해
    /// 이 windup 자세들의 Chebyshev 중심을 새 휴지 자세 후보로 잡고, 그
    /// 후보에서 **실제 임팩트 자세까지의** 최악 Δq(커밋 창에서 소화해야
    /// 하는 진짜 비용)를 비교한다. `T_w=0`은 곧 현재 방식(임팩트 자세
    /// 자체의 Chebyshev 중심)과 같아 기준선이 된다.
    ///
    /// ```text
    /// cargo test --release --lib diag_windup_rest_pose_search -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "진단 전용 — 수치를 stdout으로 뽑는다"]
    fn diag_windup_rest_pose_search() {
        let robot = crate::defaults::robot().expect("robot");
        let arm = (*robot.arm).clone();
        let n = arm.default_joints.values.len();
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let scenarios = windup_rest_pose_scenarios();

        // 현재 READY_JOINTS_4DOF 기준선.
        let current_rest = arm.default_joints.values.clone();

        for &windup_secs in &[0.0_f64, 0.06, 0.10, 0.14, 0.18, 0.22, 0.28] {
            // IK 시드가 휴지 자세에 의존하는 고정점 문제라 몇 번 반복해 수렴시킨다.
            let mut search_arm = arm.clone();
            let mut candidate = current_rest.clone();
            let mut impact_samples: Vec<Vec<f64>> = Vec::new();
            let mut solved = 0usize;
            for _iteration in 0..3 {
                let start = robot::Pose::new(rail_x, Joints::from_slice(&candidate));
                let mut lo = vec![f64::INFINITY; n];
                let mut hi = vec![f64::NEG_INFINITY; n];
                impact_samples.clear();
                solved = 0;
                for prediction in &scenarios {
                    let Ok(target) = solve_impact_target(&search_arm, prediction, &start) else {
                        continue;
                    };
                    solved += 1;
                    let q_impact = &target.pose.joints.values;
                    impact_samples.push(q_impact.clone());
                    for j in 0..n {
                        let windup = q_impact[j] - target.joint_velocities[j] * windup_secs;
                        let clamped = match search_arm.joint_limit(j) {
                            Some(limit) => windup.clamp(limit.min, limit.max),
                            None => windup,
                        };
                        lo[j] = lo[j].min(clamped);
                        hi[j] = hi[j].max(clamped);
                    }
                }
                if solved == 0 {
                    break;
                }
                candidate = chebyshev_center(&lo, &hi, &search_arm);
                search_arm.default_joints = Joints::from_slice(&candidate);
            }
            if solved == 0 {
                println!("T_w={windup_secs:.2}s: IK 해가 있는 시나리오 없음 — 탐색 불가");
                continue;
            }
            let worst = worst_dq(&candidate, &impact_samples);
            let need_secs = 1.875 * worst / arm.max_joint_speed;
            println!(
                "T_w={windup_secs:.2}s  해결={solved}/{}  후보=[{}]  \
                 최악Δq(후보→임팩트)={worst:.3} rad → 필요시간 {need_secs:.3}s",
                scenarios.len(),
                candidate
                    .iter()
                    .map(|v| format!("{v:.4}"))
                    .collect::<Vec<_>>()
                    .join(", "),
            );
        }

        // 기준선: 현재 READY_JOINTS_4DOF에서 같은 임팩트 표본까지의 최악 Δq.
        let start = robot::Pose::new(rail_x, Joints::from_slice(&current_rest));
        let mut baseline_samples: Vec<Vec<f64>> = Vec::new();
        for prediction in &scenarios {
            if let Ok(target) = solve_impact_target(&arm, prediction, &start) {
                baseline_samples.push(target.pose.joints.values.clone());
            }
        }
        let baseline_worst = worst_dq(&current_rest, &baseline_samples);
        println!(
            "\n기준선(현재 READY_JOINTS_4DOF=[{}]): 최악Δq={baseline_worst:.3} rad → 필요시간 {:.3}s",
            current_rest
                .iter()
                .map(|v| format!("{v:.4}"))
                .collect::<Vec<_>>()
                .join(", "),
            1.875 * baseline_worst / arm.max_joint_speed,
        );
    }
}
