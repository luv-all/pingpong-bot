//! 순수 물리/스윙 계획.

use nalgebra::Vector3;

use super::impact::{rally_return_velocity, required_racket_velocity};
use crate::constants::{G, table};
use crate::defaults;
use crate::error::{DomainError, SwingPlanError};
use crate::robot::Arm;
use crate::{Joints, Prediction, RailMotion, RobotPose, SwingTrajectory};

#[derive(Debug, Clone, PartialEq)]
pub struct PlannedIntercept {
    pub prediction: Prediction,
    pub trajectory: SwingTrajectory,
}

/// 공기 저항을 포함한 공 가속도 [m/s^2].
pub fn accel(velocity: Vector3<f64>, drag_coefficient: f64) -> Vector3<f64> {
    return G - drag_coefficient * velocity.norm() * velocity;
}

/// 임팩트까지 남은 시간이 스윙 commit 창 `[MIN_SWING, COMMIT_MAX]` 안인지.
///
/// 창보다 이르면 대기(발사 직후 긴 궤적 금지), 짧으면 `InsufficientTime`.
pub fn in_swing_commit_window(time_to_impact_secs: f64) -> bool {
    return (defaults::control().min_swing_secs..=defaults::control().swing_commit_max_secs).contains(&time_to_impact_secs);
}

/// 네트 통과 후인지 - ground truth/EKF control 공통 commit 게이트.
pub fn ball_past_midcourt_for_commit(ball_y: f64) -> bool {
    return ball_y <= table::LENGTH_Y * defaults::control().swing_commit_max_ball_y_frac;
}

/// 예측/현재 포즈로 quintic 스윙 궤적을 계획한다.
pub fn plan_swing(
    arm: &Arm,
    prediction: Prediction,
    start: &RobotPose,
) -> Result<SwingTrajectory, DomainError> {
    let time_to_impact = prediction.time_to_impact_secs;
    if time_to_impact < defaults::control().min_swing_secs {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::InsufficientTime {
                time_to_impact_secs: time_to_impact,
                min_swing_secs: defaults::control().min_swing_secs,
            },
        ));
    }

    let impact_position = prediction.impact_position;
    let v_in = prediction.incoming_velocity;
    let v_out = rally_return_velocity(impact_position, v_in);
    let desired_normal = (v_out - v_in).normalize();

    let ik_hint = arm
        .with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))
        .map_err(DomainError::InfeasibleSwing)?;
    let racket_center = crate::Point3::from(
        impact_position.coords
            - desired_normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z),
    );
    let solved = arm
        .inverse_pose_with_rail(
            racket_center,
            desired_normal,
            &RobotPose::new(start.rail_x, ik_hint),
        )
        .map_err(DomainError::InfeasibleSwing)?;
    if crate::planner::collision::table_penetration(arm, solved.rail_x, &solved.joints) > 1e-3 {
        return Err(DomainError::InfeasibleSwing(
            SwingPlanError::InverseKinematicsNoSolution {
                target_x: impact_position.coords.x,
                target_y: impact_position.coords.y,
                target_z: impact_position.coords.z,
            },
        ));
    }
    let pose = arm
        .forward_kinematics_with_rail(solved.rail_x, &solved.joints)
        .ok_or(DomainError::InfeasibleSwing(
            SwingPlanError::InverseKinematicsNoSolution {
                target_x: prediction.impact_position.coords.x,
                target_y: prediction.impact_position.coords.y,
                target_z: prediction.impact_position.coords.z,
            },
        ))?;

    let v_r = required_racket_velocity(v_in, v_out, pose.normal, defaults::impact().racket_effective_restitution)
        .map_err(DomainError::InfeasibleSwing)?;

    let start_velocity = vec![0.0; start.joints.values.len()];
    let (rail_end_velocity, end_velocity) = arm
        .velocities_for_racket_velocity(&solved, v_r)
        .map_err(DomainError::InfeasibleSwing)?;
    let rail_motion = RailMotion {
        start: start.rail_x,
        end: solved.rail_x,
        start_velocity: 0.0,
        end_velocity: rail_end_velocity,
    };

    return build_feasible_trajectory(
        arm,
        &start.joints,
        solved.joints,
        start_velocity,
        end_velocity,
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

/// 스윙 뒤 항상 시도할 최소 복귀 시간 [s].
const RETURN_TO_CENTER_MIN_SECS: f64 = 0.3;
/// 이 시간까지 늘려도 실현 가능한 궤적이 없으면 포기한다.
const RETURN_TO_CENTER_MAX_SECS: f64 = 3.0;
/// 실패할 때마다 소요 시간을 이 배수로 늘린다.
const RETURN_TO_CENTER_GROWTH: f64 = 1.4;

/// 스윙(혹은 랠리) 뒤 로봇을 중앙 포즈(관절 `default_joints`, 레일 `default_x`
/// = 테이블 폭 중앙)로 되돌리는 궤적을 계획한다.
///
/// 레일의 `home_x`(원점, x=0)는 "대기 위치"일 뿐 테이블 중앙이 아니다 —
/// 여기서 되돌아갈 곳은 `LinearRail::default_x()`(`(x_min+x_max)*0.5`), 즉
/// 테이블 폭 한가운데다. 실제 로봇은 모터 토크 한계 때문에 레일 한쪽
/// 끝에서 반대쪽 끝으로 급하게 움직이는 궤적을 못 만든다 — 매 스윙 뒤 항상
/// 중앙으로 복귀시켜 다음 스윙의 시작 조건을 일정하게 유지한다. 볼 예측이
/// 없으므로 `plan_swing`과 달리 목표 소요 시간이 정해져 있지 않다 — 관절·
/// 레일 속도/가속/토크 한계(`trajectory_within_limits`)를 만족할 때까지
/// 소요 시간을 점진적으로 늘려가며 찾는다.
pub fn plan_return_to_center(arm: &Arm, start: &RobotPose) -> Result<SwingTrajectory, DomainError> {
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
) -> Result<SwingTrajectory, SwingPlanError> {
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
    if !trajectory_within_limits(arm, &trajectory) {
        return Err(SwingPlanError::InverseKinematicsNoSolution {
            target_x: fitted_rail.end,
            target_y: 0.0,
            target_z: table::SURFACE_Z,
        });
    }
    if !trajectory_collision_free(arm, &trajectory) {
        return Err(SwingPlanError::InverseKinematicsNoSolution {
            target_x: fitted_rail.end,
            target_y: 0.0,
            target_z: table::SURFACE_Z,
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
) -> SwingTrajectory {
    let follow_time = defaults::control().swing_follow_through_secs;
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
    return SwingTrajectory::with_follow_through(
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

fn trajectory_collision_free(arm: &Arm, trajectory: &SwingTrajectory) -> bool {
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

fn torques_within_limits(trajectory: &SwingTrajectory) -> bool {
    let control = defaults::control();
    let inertia = control.joint_inertia;
    let peaks = trajectory.peak_joint_accelerations();
    return peaks.iter().enumerate().all(|(index, &alpha)| {
        let limit = control
            .max_joint_torques
            .get(index)
            .copied()
            .unwrap_or(0.0);
        return inertia * alpha <= limit;
    });
}

fn peak_torque_scale(trajectory: &SwingTrajectory) -> f64 {
    let control = defaults::control();
    let inertia = control.joint_inertia;
    let mut scale = 1.0_f64;
    for (index, &alpha) in trajectory.peak_joint_accelerations().iter().enumerate() {
        let limit = control
            .max_joint_torques
            .get(index)
            .copied()
            .unwrap_or(0.0);
        let required = inertia * alpha;
        if required > limit && required > f64::EPSILON {
            scale = scale.min(limit / required * 0.95);
        }
    }
    return scale;
}

fn trajectory_within_limits(arm: &Arm, trajectory: &SwingTrajectory) -> bool {
    let joints_ok = trajectory.peak_joint_speed() <= arm.max_joint_speed
        && trajectory.peak_joint_acceleration() <= defaults::control().max_joint_accel
        && torques_within_limits(trajectory);
    let rail_ok = arm
        .rail
        .as_ref()
        .map_or(true, |rail| trajectory.peak_rail_speed() <= rail.max_speed);
    if !joints_ok || !rail_ok {
        return false;
    }
    let samples = (trajectory.duration_secs / 0.002).ceil() as usize;
    for index in 0..=samples.max(1) {
        let time = trajectory.duration_secs * index as f64 / samples.max(1) as f64;
        if !arm.joints_in_limits(&trajectory.sample_at(time)) {
            return false;
        }
        if let Some(rail) = &arm.rail {
            let x = trajectory.sample_rail_at(time);
            if !(rail.x_min..=rail.x_max).contains(&x) {
                return false;
            }
        }
    }
    return true;
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
        if trajectory_within_limits(arm, &trajectory) {
            return (end_velocity, rail);
        }

        let peak_speed = trajectory.peak_joint_speed();
        let peak_accel = trajectory.peak_joint_acceleration();
        let speed_scale = if peak_speed > arm.max_joint_speed {
            arm.max_joint_speed / peak_speed * 0.95
        } else {
            1.0
        };
        let accel_scale = if peak_accel > defaults::control().max_joint_accel {
            defaults::control().max_joint_accel / peak_accel * 0.95
        } else {
            1.0
        };
        let torque_scale = peak_torque_scale(&trajectory);
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
        return crate::defaults::arm().expect("테스트용 4DOF arm");
    }

    fn sample_start(arm: &Arm) -> RobotPose {
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        return RobotPose::new(rail_x, arm.default_joints.clone());
    }

    fn sample_prediction(time_to_impact_secs: f64) -> Prediction {
        let arm = sample_three_dof_arm();
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let impact_position = arm
            .forward_kinematics_with_rail(rail_x, &arm.default_joints)
            .expect("기본 자세 FK")
            .position;
        return Prediction {
            time_to_impact_secs,
            impact_position,
            incoming_velocity: Vector3::new(0.0, -4.0, -0.2),
        };
    }

    #[test]
    fn in_swing_commit_window_bounds() {
        assert!(!in_swing_commit_window(0.05));
        assert!(in_swing_commit_window(0.12));
        assert!(in_swing_commit_window(defaults::control().swing_commit_max_secs));
        assert!(!in_swing_commit_window(defaults::control().swing_commit_max_secs + 0.01));
    }

    #[test]
    fn midcourt_gate_matches_fraction() {
        let limit = table::LENGTH_Y * defaults::control().swing_commit_max_ball_y_frac;
        assert!(!ball_past_midcourt_for_commit(limit + 0.01));
        assert!(ball_past_midcourt_for_commit(limit));
        assert!(ball_past_midcourt_for_commit(0.3));
    }

    #[test]
    fn plan_swing_reaches_impact_with_end_velocity() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let prediction = sample_prediction(0.35);
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
            (rally_return_velocity(prediction.impact_position, prediction.incoming_velocity)
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
        let desired_racket_velocity = required_racket_velocity(
            prediction.incoming_velocity,
            rally_return_velocity(prediction.impact_position, prediction.incoming_velocity),
            pose.normal,
            defaults::impact().racket_effective_restitution,
        )
        .expect("required racket velocity");
        assert!(
            (actual_racket_velocity - desired_racket_velocity).norm() < 0.05,
            "actual={actual_racket_velocity:?}, desired={desired_racket_velocity:?}, joint_speed={}, joint_accel={}, rail_speed={}",
            trajectory.peak_joint_speed(),
            trajectory.peak_joint_acceleration(),
            trajectory.peak_rail_speed(),
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
    fn plan_swing_moves_rail_to_impact_x() {
        let arm = sample_three_dof_arm();
        let start = RobotPose::new(0.1, arm.default_joints.clone());
        let reachable = arm
            .forward_kinematics_with_rail(table::WIDTH_X * 0.8, &arm.default_joints)
            .expect("FK")
            .position;
        let impact = crate::Point3::new(reachable.coords.x, reachable.coords.y, reachable.coords.z);
        let prediction = Prediction {
            time_to_impact_secs: 0.3,
            impact_position: impact,
            incoming_velocity: Vector3::new(0.0, -5.0, -0.2),
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
    fn best_swing_rejects_clamped_contact_and_selects_reachable_candidate() {
        let arm = sample_three_dof_arm();
        let start = sample_start(&arm);
        let reachable = sample_prediction(0.18);
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
        assert!((min_swing_secs - defaults::control().min_swing_secs).abs() < f64::EPSILON);
    }

    #[test]
    fn competition_geometry_reachable_with_rail() {
        let arm = crate::defaults::arm().expect("competition arm");

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
        assert!(!trajectory_within_limits(&arm, &trajectory));
    }
}
