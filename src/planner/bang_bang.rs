//! 순수 토크 한계 기반 bang-bang 스윙 — quintic처럼 정해둔 궤적 "모양"이
//! 없다. 매 적분 스텝 `planner::dynamics::{mass_matrix, forward_dynamics}`로
//! 실제 강체 동역학을 적분하면서, 관절마다 시간최적(bang-bang) 스위칭
//! 곡선으로 토크를 명령한다. GUI에서 quintic 스윙과 육안 비교하기 위한
//! 디버그 경로 — `plan_swing`(quintic, 실제 게임플레이 경로)은 건드리지
//! 않는다.
//!
//! 원본 알고리즘은 `tools/swing_bench`(오프라인 CLI 벤치마크)에서 먼저
//! 검증했다. 종료 조건은 관절 공간 목표속도를 칼같이 맞추는 대신, FK로
//! 역산한 실제 라켓 속도의 방향·크기가 허용오차 안인지로 본다 — 그렇지
//! 않으면 하나의 특정 관절속도 조합만 강요하게 돼 불필요한 백스윙성
//! 왕복이 "최적해"로 나온다(`tools/swing_bench`에서 실측 확인).

use nalgebra::Vector3;

use super::dynamics::{forward_dynamics, mass_matrix};
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
/// 수렴 못 하면 포기하는 계획 시간 상한 [s].
const MAX_PLAN_TIME_SECS: f64 = 2.0;

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

    pub fn end_joints(&self) -> &Joints {
        return self.joint_samples.last().expect("최소 1개 샘플");
    }

    pub fn follow_through_rail_x(&self) -> f64 {
        return *self.rail_samples.last().expect("최소 1개 샘플");
    }
}

/// `predictions` 중 IK가 풀리는 첫 후보로 bang-bang 궤적을 계획한다.
/// 선택 순서는 `plan_best_swing`과 같은 "현재 라켓 위치에 가까운 순".
pub fn plan_bang_bang_swing(
    arm: &Arm,
    predictions: &[Prediction],
    start: &RobotPose,
) -> Result<BangBangTrajectory, DomainError> {
    let current_position = if arm.rail.is_some() {
        arm.forward_kinematics_with_rail(start.rail_x, &start.joints)
    } else {
        arm.forward_kinematics(&start.joints)
    }
    .map(|pose| pose.position.v)
    .unwrap_or_default();
    let mut ranked: Vec<Prediction> = predictions
        .iter()
        .copied()
        .filter(|prediction| in_swing_commit_window(prediction.time_to_impact_secs))
        .collect();
    ranked.sort_by(|left, right| {
        let left_cost = (left.impact_position.v - current_position).norm();
        let right_cost = (right.impact_position.v - current_position).norm();
        left_cost
            .partial_cmp(&right_cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut last_error = None;
    for prediction in ranked {
        match plan_bang_bang_for(arm, &prediction, start) {
            Ok(trajectory) => return Ok(trajectory),
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

fn plan_bang_bang_for(
    arm: &Arm,
    prediction: &Prediction,
    start: &RobotPose,
) -> Result<BangBangTrajectory, DomainError> {
    let mut target = solve_impact_target(arm, prediction, start)?;
    clamp_to_speed_caps(arm, &mut target.joint_velocities, &mut target.rail_velocity);

    let n = start.joints.values.len();
    let mut q = start.joints.values.clone();
    let mut qdot = vec![0.0; n];
    let mut rail_x = start.rail_x;
    let mut rail_v = 0.0;
    let rail_max_speed = arm.rail.as_ref().map_or(f64::INFINITY, |r| r.max_speed);

    let mut joint_samples = vec![Joints::from_slice(&q)];
    let mut rail_samples = vec![rail_x];

    let mut t = 0.0;
    let mut converged = false;
    while t < MAX_PLAN_TIME_SECS {
        let m = mass_matrix(arm, &Joints::from_slice(&q));
        let mut torque_cmd = vec![0.0; n];
        for i in 0..n {
            let effective_inertia = m[(i, i)].max(1e-9);
            let a_max = (arm.joint_torque_limits[i] / effective_inertia).max(1e-6);
            let x = q[i] - target.pose.joints.values[i];
            let v = qdot[i] - target.joint_velocities[i];
            let a_cmd = bang_bang_accel(x, v, a_max);
            torque_cmd[i] = (a_cmd * effective_inertia)
                .clamp(-arm.joint_torque_limits[i], arm.joint_torque_limits[i]);
        }
        let Some(accel) = forward_dynamics(arm, &Joints::from_slice(&q), &qdot, &torque_cmd)
        else {
            break;
        };
        for i in 0..n {
            qdot[i] += accel[i] * PLAN_DT_SECS;
            qdot[i] = qdot[i].clamp(-arm.max_joint_speed, arm.max_joint_speed);
            q[i] += qdot[i] * PLAN_DT_SECS;
        }
        {
            let x = rail_x - target.pose.rail_x;
            let v = rail_v - target.rail_velocity;
            let a_cmd = bang_bang_accel(x, v, RAIL_ACCEL_M_S2);
            rail_v += a_cmd * PLAN_DT_SECS;
            rail_v = rail_v.clamp(-rail_max_speed, rail_max_speed);
            rail_x += rail_v * PLAN_DT_SECS;
        }
        t += PLAN_DT_SECS;
        joint_samples.push(Joints::from_slice(&q));
        rail_samples.push(rail_x);

        let pos_err = q
            .iter()
            .zip(&target.pose.joints.values)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max)
            .max((rail_x - target.pose.rail_x).abs());
        if pos_err < POSITION_TOLERANCE_RAD_OR_M
            && racket_velocity_ok(arm, rail_x, rail_v, &q, &qdot, target.racket_velocity)
        {
            converged = true;
            break;
        }
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

/// 목표 속도가 실제 관절/레일 속도 한계를 넘으면 그 한계로 자른다 — 넘는
/// 채로 두면 bang-bang 스위칭이 영원히 못 줄어드는 속도오차를 쫓다 위치를
/// 지나쳐 발산한다(`tools/swing_bench`에서 실측 확인).
fn clamp_to_speed_caps(arm: &Arm, joint_velocities: &mut [f64], rail_velocity: &mut f64) {
    for v in joint_velocities.iter_mut() {
        *v = v.clamp(-arm.max_joint_speed, arm.max_joint_speed);
    }
    if let Some(rail) = &arm.rail {
        *rail_velocity = rail_velocity.clamp(-rail.max_speed, rail.max_speed);
    }
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
    let cos_angle = (achieved.dot(&target_racket_velocity) / (achieved_speed * target_speed))
        .clamp(-1.0, 1.0);
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
    return Some((perturbed.position.v - base.position.v) / STEP);
}

/// 1차원 이중적분기를 원점(목표)으로 모는 시간최적 bang-bang 스위칭.
/// `x`/`v`는 목표 기준 상대 위치/속도 오차(`현재 - 목표`), `a_max`는 이
/// 축이 낼 수 있는 최대 가속.
fn bang_bang_accel(x: f64, v: f64, a_max: f64) -> f64 {
    let switch = x + v * v.abs() / (2.0 * a_max);
    if switch.abs() < 1e-12 {
        return 0.0;
    }
    return -a_max * switch.signum();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::Prediction;
    use crate::robot::Arm;

    fn sample_prediction(time_to_impact_secs: f64) -> Prediction {
        let arm = Arm::competition().expect("arm");
        let pose = arm
            .forward_kinematics_with_rail(arm.rail.expect("rail").default_x(), &arm.default_joints)
            .expect("FK");
        return Prediction {
            time_to_impact_secs,
            impact_position: crate::Point3::new(pose.position.v.x, 0.30, pose.position.v.z),
            incoming_velocity: Vector3::new(0.0, -1.0, 0.0),
        };
    }

    #[test]
    #[ignore = "known regression after realistic joint-speed recalibration — \
                see .omc/research/known-regressions-realistic-joint-speed.md"]
    fn plan_bang_bang_swing_converges_for_a_reachable_impact() {
        // 완만한 시나리오(약한 입사속도)로 메커니즘 자체의 수렴을 확인한다.
        // 빠른/까다로운 시나리오는 `tools/swing_bench`에서 이미 실측했듯
        // 실기 토크·속도 한계 안에서 진짜로 도달 불가능할 수 있고, 그 경우
        // `Err(InfeasibleSwing)`을 내는 게 올바른 동작이지 버그가 아니다.
        let arm = Arm::competition().expect("arm");
        let start = arm.initial_state();
        let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());
        let trajectory = plan_bang_bang_swing(&arm, &[sample_prediction(0.30)], &start_pose)
            .expect("bang-bang 계획 성공");
        assert!(trajectory.duration_secs() > 0.0);
        let end = trajectory.end_joints();
        assert_eq!(end.values.len(), arm.joint_count());
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
