//! 조그 입력 → 궤적 조합. 툴에서 `robot::motion::Trajectory`를 만든다.

mod draft;
mod kind;

use anyhow::{Context, Result, ensure};
use nalgebra::{Rotation3, Vector3};
use pingpong_bot::Point3;
use pingpong_bot::defaults::{ControlParams, ImpactParams};
use pingpong_bot::estimator::Impact;
use pingpong_bot::robot::motion;
use pingpong_bot::robot::{self, Arm, Joints, RacketPose};

pub use draft::Draft;
pub use kind::Kind;

/// 4-dof 관절 표시 이름 — sim joint windows 오버레이와 동일 (`j0`…`j3`).
pub const JOINT_LABELS: [&str; 4] = ["j0", "j1", "j2", "j3"];

/// 라켓 목표: 현재 FK 기준 상대 이동 한계 [m].
pub const REACH_DELTA_M: f64 = 0.12;

pub fn joint_label(index: usize) -> String {
    return JOINT_LABELS
        .get(index)
        .map(|s| (*s).to_string())
        .unwrap_or_else(|| format!("j{index}"));
}

pub fn compose(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
    duration_secs: f64,
    max_delta_deg: f64,
) -> Result<motion::Trajectory> {
    return match draft.kind {
        Kind::Joint => {
            ensure!(
                draft.joint_index < start.joints.values.len(),
                "joint out of range"
            );
            ensure!(draft.joint_deg.is_finite(), "angle finite");
            let mut target = start.joints.values.clone();
            target[draft.joint_index] = draft.joint_deg.to_radians();
            move_traj(
                arm,
                start,
                Joints::from_slice(&target),
                start.rail_x,
                duration_secs,
                max_delta_deg,
            )
        }
        Kind::Angles => {
            ensure!(
                draft.angles_deg.len() == start.joints.values.len(),
                "need {} angles",
                start.joints.values.len()
            );
            for d in &draft.angles_deg {
                ensure!(d.is_finite(), "angle finite");
            }
            let rads: Vec<f64> = draft.angles_deg.iter().map(|d| d.to_radians()).collect();
            move_traj(
                arm,
                start,
                Joints::from_slice(&rads),
                start.rail_x,
                duration_secs,
                max_delta_deg,
            )
        }
        Kind::RailAbs => {
            ensure!(draft.rail_x.is_finite(), "rail finite");
            move_traj(
                arm,
                start,
                start.joints.clone(),
                draft.rail_x,
                duration_secs,
                max_delta_deg,
            )
        }
        Kind::Ik => {
            let target = reach_target(arm, start, draft.reach_dxyz)?;
            let linear = arm
                .rail
                .ok_or_else(|| anyhow::anyhow!("arm has no linear rail"))?;
            let joints = arm
                .inverse_kinematics_with_rail(&linear, start.rail_x, target, Some(&start.joints))
                .context("ik")?;
            move_traj(
                arm,
                start,
                joints,
                start.rail_x,
                duration_secs,
                max_delta_deg,
            )
        }
        Kind::Pose => {
            let (target, normal) = reach_pose_target(arm, start, draft)?;
            let solved = arm
                .inverse_pose_with_rail(target, normal, start)
                .context("pose ik")?;
            move_traj(
                arm,
                start,
                solved.joints.clone(),
                solved.rail_x,
                duration_secs,
                max_delta_deg,
            )
        }
        Kind::Swing => swing_traj(arm, start, draft, duration_secs, max_delta_deg),
        Kind::AimBall => {
            let (target, normal) = ball_aim_target(arm, start, draft)?;
            let solved = arm
                .inverse_pose_with_rail(target, normal, start)
                .context("aim ik")?;
            move_traj(
                arm,
                start,
                solved.joints.clone(),
                solved.rail_x,
                duration_secs,
                max_delta_deg,
            )
        }
        Kind::SwingBall => swing_ball_traj(arm, start, draft, duration_secs, max_delta_deg),
    };
}

/// 미리보기용: 상대 목표가 IK로 풀리는지.
pub fn reach_ok(arm: &Arm, start: &robot::Pose, draft: &Draft) -> bool {
    return match draft.kind {
        Kind::Ik => {
            let Ok(target) = reach_target(arm, start, draft.reach_dxyz) else {
                return false;
            };
            let Some(linear) = arm.rail else {
                return false;
            };
            arm.inverse_kinematics_with_rail(&linear, start.rail_x, target, Some(&start.joints))
                .is_ok()
        }
        Kind::Pose => {
            let Ok((target, normal)) = reach_pose_target(arm, start, draft) else {
                return false;
            };
            arm.inverse_pose_with_rail(target, normal, start).is_ok()
        }
        Kind::Swing => {
            let Ok((target, normal)) = reach_pose_target(arm, start, draft) else {
                return false;
            };
            arm.inverse_pose_with_rail(target, normal, start).is_ok()
        }
        Kind::AimBall | Kind::SwingBall => {
            let Ok((target, normal)) = ball_aim_target(arm, start, draft) else {
                return false;
            };
            arm.inverse_pose_with_rail(target, normal, start).is_ok()
        }
        _ => true,
    };
}

fn swing_ball_traj(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
    duration_secs: f64,
    max_delta_deg: f64,
) -> Result<motion::Trajectory> {
    let (target, aim_normal) = ball_aim_target(arm, start, draft)?;
    let v_in = vec3(draft.ball_vin)?;
    ensure!(v_in.norm() > 1e-3, "공 속도가 너무 작음");

    let impact = arm
        .inverse_pose_with_rail(target, aim_normal, start)
        .context("swing-ball pose ik")?;
    let racket = arm
        .forward_kinematics_with_rail(impact.rail_x, &impact.joints)
        .context("fk at impact")?;
    let normal = racket.normal.normalize();

    let v_out = Impact::rally_return(target, v_in);
    let e = ImpactParams::default().racket_effective_restitution;
    let v_r = Impact::required_racket_velocity(v_in, v_out, normal, e).context("라켓 속도 역산")?;

    let (rail_impact_vel, joint_impact_vel) = arm
        .velocities_for_racket_velocity(&impact, v_r)
        .context("joint velocities for racket speed")?;

    ensure_max_delta(&start.joints, &impact.joints, max_delta_deg)?;
    return build_follow_through_swing(
        start,
        &impact,
        joint_impact_vel,
        rail_impact_vel,
        duration_secs,
    );
}

fn swing_traj(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
    duration_secs: f64,
    max_delta_deg: f64,
) -> Result<motion::Trajectory> {
    ensure!(
        draft.swing_speed.is_finite() && draft.swing_speed > 0.0,
        "speed > 0"
    );
    let (target, normal) = reach_pose_target(arm, start, draft)?;
    let impact = arm
        .inverse_pose_with_rail(target, normal, start)
        .context("swing pose ik")?;

    let racket = arm
        .forward_kinematics_with_rail(impact.rail_x, &impact.joints)
        .context("fk at impact")?;
    let normal = racket.normal.normalize();
    let v_r = normal * draft.swing_speed;
    let (rail_impact_vel, joint_impact_vel) = arm
        .velocities_for_racket_velocity(&impact, v_r)
        .context("joint velocities for racket speed")?;

    ensure_max_delta(&start.joints, &impact.joints, max_delta_deg)?;
    return build_follow_through_swing(
        start,
        &impact,
        joint_impact_vel,
        rail_impact_vel,
        duration_secs,
    );
}

fn build_follow_through_swing(
    start: &robot::Pose,
    impact: &robot::Pose,
    joint_impact_vel: Vec<f64>,
    rail_impact_vel: f64,
    duration_secs: f64,
) -> Result<motion::Trajectory> {
    let follow = ControlParams::default().swing_follow_through_secs.max(0.02);
    let approach = duration_secs.max(follow + 0.05);
    let impact_time = (approach - follow).max(0.05);
    let duration = impact_time + follow;

    let n = impact.joints.values.len();
    let mut follow_joints = Vec::with_capacity(n);
    for i in 0..n {
        follow_joints.push(impact.joints.values[i] + joint_impact_vel[i] * follow);
    }
    let follow_rail = impact.rail_x + rail_impact_vel * follow;
    let start_vel = vec![0.0; n];
    let follow_vel = vec![0.0; n];

    return Ok(motion::Trajectory::with_follow_through(
        start.joints.clone(),
        impact.joints.clone(),
        Joints::from_slice(&follow_joints),
        start_vel,
        joint_impact_vel,
        follow_vel,
        impact_time,
        duration,
        motion::Rail {
            start: start.rail_x,
            end: impact.rail_x,
            start_velocity: 0.0,
            end_velocity: rail_impact_vel,
        },
        follow_rail,
        0.0,
    ));
}

/// 도달점 + (입사 반대 / 현재 법선) 기울기.
fn ball_aim_target(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
) -> Result<(Point3, Vector3<f64>)> {
    let target = point3(draft.arrival_xyz)?;
    let base_normal = {
        let vin = Vector3::new(draft.ball_vin[0], draft.ball_vin[1], draft.ball_vin[2]);
        if draft.kind == Kind::SwingBall && vin.norm() > 1e-3 {
            -vin.normalize()
        } else if let Ok(racket) = current_racket(arm, start) {
            racket.normal.normalize()
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        }
    };
    let normal = tilt_normal(base_normal, draft.tilt_pitch_deg, draft.tilt_yaw_deg)?;
    return Ok((target, normal));
}

fn current_racket(arm: &Arm, start: &robot::Pose) -> Result<RacketPose> {
    return arm
        .forward_kinematics_with_rail(start.rail_x, &start.joints)
        .context("현재 라켓 FK");
}

fn reach_target(arm: &Arm, start: &robot::Pose, dxyz: [f64; 3]) -> Result<Point3> {
    for v in dxyz {
        ensure!(v.is_finite(), "finite");
    }
    let racket = current_racket(arm, start)?;
    let p = racket.position.coords;
    return Ok(Point3::new(p.x + dxyz[0], p.y + dxyz[1], p.z + dxyz[2]));
}

fn reach_pose_target(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
) -> Result<(Point3, Vector3<f64>)> {
    let racket = current_racket(arm, start)?;
    let p = racket.position.coords;
    let target = Point3::new(
        p.x + draft.reach_dxyz[0],
        p.y + draft.reach_dxyz[1],
        p.z + draft.reach_dxyz[2],
    );
    let normal = tilt_normal(racket.normal, draft.tilt_pitch_deg, draft.tilt_yaw_deg)?;
    return Ok((target, normal));
}

/// 현재 법선을 pitch(전후 기울기)·yaw(좌우 기울기)로 기울인다.
fn tilt_normal(base: Vector3<f64>, pitch_deg: f64, yaw_deg: f64) -> Result<Vector3<f64>> {
    ensure!(pitch_deg.is_finite() && yaw_deg.is_finite(), "finite");
    let n0 = base.normalize();
    let pitch = pitch_deg.to_radians();
    let yaw = yaw_deg.to_radians();
    // 월드 X = 좌우, Y = 전후. pitch는 X축 회전, yaw는 Z축 회전.
    let after_pitch = Rotation3::from_axis_angle(&Vector3::x_axis(), pitch) * n0;
    let after_yaw = Rotation3::from_axis_angle(&Vector3::z_axis(), yaw) * after_pitch;
    let n = after_yaw.normalize();
    ensure!(n.norm() > 1e-6, "normal");
    return Ok(n);
}

fn move_traj(
    arm: &Arm,
    start: &robot::Pose,
    target_joints: Joints,
    target_rail: f64,
    duration_secs: f64,
    max_delta_deg: f64,
) -> Result<motion::Trajectory> {
    ensure_rail_in_range(arm, target_rail)?;
    ensure_max_delta(&start.joints, &target_joints, max_delta_deg)?;
    let n = target_joints.values.len();
    return Ok(motion::Trajectory::new(
        start.joints.clone(),
        target_joints,
        vec![0.0; n],
        vec![0.0; n],
        duration_secs,
        motion::Rail {
            start: start.rail_x,
            end: target_rail,
            start_velocity: 0.0,
            end_velocity: 0.0,
        },
    ));
}

fn ensure_rail_in_range(arm: &Arm, rail_x: f64) -> Result<()> {
    ensure!(rail_x.is_finite(), "rail finite");
    let Some(rail) = arm.rail.as_ref() else {
        return Ok(());
    };
    ensure!(
        rail_x >= rail.x_min && rail_x <= rail.x_max,
        "rail {:.3} out of range [{:.3}, {:.3}]",
        rail_x,
        rail.x_min,
        rail.x_max
    );
    return Ok(());
}

fn ensure_max_delta(from: &Joints, to: &Joints, max_delta_deg: f64) -> Result<()> {
    let max_delta = max_delta_deg.to_radians();
    for (index, (a, b)) in from.values.iter().zip(&to.values).enumerate() {
        ensure!(
            (b - a).abs() <= max_delta,
            "joint {index} Δ {:.1}° > maxdelta {}",
            (b - a).abs().to_degrees(),
            max_delta_deg
        );
    }
    return Ok(());
}

fn point3(v: [f64; 3]) -> Result<Point3> {
    let vec = vec3(v)?;
    return Ok(Point3::new(vec.x, vec.y, vec.z));
}

fn vec3(v: [f64; 3]) -> Result<Vector3<f64>> {
    ensure!(
        v[0].is_finite() && v[1].is_finite() && v[2].is_finite(),
        "finite"
    );
    return Ok(Vector3::new(v[0], v[1], v[2]));
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::defaults;

    #[test]
    fn joint_preview_respects_maxdelta() {
        let built = defaults::robot().expect("robot");
        let start = robot::Pose::new(0.0, built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Joint;
        draft.joint_index = 0;
        draft.joint_deg = start.joints.values[0].to_degrees() + 40.0;
        let err = compose(&built.arm, &start, &draft, 1.0, 15.0).unwrap_err();
        assert!(format!("{err:#}").contains("maxdelta"));
    }

    #[test]
    fn zero_reach_delta_is_reachable() {
        let built = defaults::robot().expect("robot");
        let start = robot::Pose::new(0.0, built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Ik;
        assert!(reach_ok(&built.arm, &start, &draft));
    }
}
