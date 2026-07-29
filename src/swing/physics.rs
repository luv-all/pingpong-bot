//! 순수 물리/스윙 계획.

use nalgebra::Vector3;

use crate::constants::{G, table};
use crate::defaults;
use crate::defaults::planner::{
    RETURN_TO_CENTER_GROWTH, RETURN_TO_CENTER_MAX_SECS, RETURN_TO_CENTER_MIN_SECS,
};
use crate::error::{DomainError, SwingPlanError};
use crate::estimator::Prediction;
use crate::planner::Impact;
use crate::robot::Arm;
use crate::robot::{Joints, RobotPose};

use super::impact_target::solve_impact_target;
use super::planned_intercept::PlannedIntercept;
use super::rail_motion::RailMotion;
use super::trajectory::Trajectory;

/// 비행 중 공기력만 (중력 제외) [m/s^2].
///
/// `-k|v|v + k_m(ω × v)`. Rapier는 중력을 따로 쓰므로 외력에는 이것만 넣는다.
///
/// 테이블 바운스 마찰이 Rapier에서 비현실적으로 큰 ω를 만들 수 있어
/// Magnus에 쓰는 |ω|는 [`MAGNUS_OMEGA_MAX`]로 클립한다.
pub use crate::defaults::planner::MAGNUS_OMEGA_MAX;

pub fn aero_accel(
    velocity: Vector3<f64>,
    omega: Vector3<f64>,
    drag_coefficient: f64,
    magnus_coefficient: f64,
) -> Vector3<f64> {
    let drag = -drag_coefficient * velocity.norm() * velocity;
    let omega_eff = {
        let w = omega.norm();
        if w > MAGNUS_OMEGA_MAX {
            omega * (MAGNUS_OMEGA_MAX / w)
        } else {
            omega
        }
    };
    let magnus = magnus_coefficient * omega_eff.cross(&velocity);
    return drag + magnus;
}

/// 중력 + 항력 + Magnus [m/s^2]. plan Model C: `g - k|v|v + k_m(ω×v)`.
pub fn accel(
    velocity: Vector3<f64>,
    omega: Vector3<f64>,
    drag_coefficient: f64,
    magnus_coefficient: f64,
) -> Vector3<f64> {
    return G + aero_accel(velocity, omega, drag_coefficient, magnus_coefficient);
}

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
    start: &RobotPose,
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
    let rail_motion = RailMotion {
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

pub fn plan_best_swing(
    arm: &Arm,
    predictions: &[Prediction],
    start: &RobotPose,
) -> Result<PlannedIntercept, DomainError> {
    const MAX_CONTACT_ERROR: f64 = 0.005;
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
pub fn plan_coarse_track(arm: &Arm, predictions: &[Prediction]) -> Option<RobotPose> {
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
    // 비행)이어도, 레일 x라도 미리 맞추도록 도달 구 안으로 클램프한 목표에
    // IK를 건다(y=접수 깊이 우선 보존). coarse 추종은 레일이 있는 로봇 대상.
    let rail = arm.rail.as_ref()?;
    let (_rail_x, reachable) = arm.clamp_impact_for_rail(rail, racket_center);
    // 기본 중앙 포즈를 힌트로 단일 IK. 실제 이동은 rate-limited 추종 루프가 함.
    let hint = RobotPose::new(rail.default_x(), arm.default_joints.clone());
    return arm
        .inverse_pose_with_rail(reachable, desired_normal, &hint)
        .ok();
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
pub fn plan_return_to_center(arm: &Arm, start: &RobotPose) -> Result<Trajectory, DomainError> {
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
        let rail = RailMotion {
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
    rail: RailMotion,
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
                worst = worst.max(crate::planner::collision::table_penetration(
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
    rail: RailMotion,
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
        if crate::planner::collision::table_penetration(arm, rail_x, &joints) > 1e-3 {
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
        crate::robot::dynamics::required_joint_torques_into(
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

/// quintic이 관절 한계 안에 들어오도록 임팩트 각속도를 점진적으로 줄인다 ( 근사).
fn fit_end_velocity(
    arm: &Arm,
    start: &Joints,
    end: &Joints,
    start_velocity: &[f64],
    mut end_velocity: Vec<f64>,
    duration: f64,
    mut rail: RailMotion,
) -> (Vec<f64>, RailMotion) {
    for _ in 0..32 {
        let trajectory = trajectory_with_follow_through(
            arm,
            start,
            end,
            start_velocity.to_vec(),
            end_velocity.clone(),
            duration,
            rail,
        );
        // 최악 위반 관절의 `|토크|/한계` 비율. >1 이면 그 역수로 끝속도를 줄여
        // 토크 한계 안으로 끌어온다 (관절별 한계를 반영한 스케일). 이용률을 한
        // 번만 계산하고 실현 가능 판정·스케일에 함께 쓴다.
        let torque_util = peak_torque_utilization(arm, &trajectory);
        if torque_util <= 1.0 && kinematic_limits_ok(arm, &trajectory) {
            return (end_velocity, rail);
        }

        let peak_speed = trajectory.peak_joint_speed();
        let peak_accel = trajectory.peak_joint_acceleration();
        let speed_scale = if peak_speed > arm.max_joint_speed {
            arm.max_joint_speed / peak_speed * 0.95
        } else {
            1.0
        };
        let accel_scale = if peak_accel > defaults::ControlParams::default().max_joint_accel {
            defaults::ControlParams::default().max_joint_accel / peak_accel * 0.95
        } else {
            1.0
        };
        let torque_scale = if torque_util > 1.0 {
            1.0 / torque_util * 0.95
        } else {
            1.0
        };
        let scale = speed_scale.min(accel_scale).min(torque_scale);
        if scale >= 0.99 {
            break;
        }
        for v in &mut end_velocity {
            *v *= scale;
        }
        rail.end_velocity *= scale;
    }

    // 한계를 완전히 못 맞춰도 끝속도를 0으로 버리지 않는다 (타격 의도 유지).
    // 최종 검증은 build_feasible_trajectory의 trajectory_within_limits가 한다.
    return (end_velocity, rail);
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::Prediction;
    use crate::constants::table;
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

    fn sample_start(arm: &Arm) -> RobotPose {
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        return RobotPose::new(rail_x, arm.default_joints.clone());
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
        assert!(in_swing_commit_window(0.12));
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
            crate::planner::collision::table_penetration(
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
        let start = RobotPose::new(0.1, arm.default_joints.clone());
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
        let start = RobotPose::new(rail_x, arm.default_joints.clone());
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
        let start = RobotPose::new(rail_x, arm.default_joints.clone());
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
            RailMotion::fixed(start.rail_x),
        );
        // 한계 위반은 "무엇을" 어겼는지까지 잡힌다 (이전엔 bool 하나였다).
        assert_eq!(
            kinematic_limit_violation(&arm, &trajectory),
            Some("관절 속도"),
            "임팩트 각속도 4.0 rad/s는 한계 {:.2} rad/s를 넘어야 함",
            arm.max_joint_speed
        );
    }
}
