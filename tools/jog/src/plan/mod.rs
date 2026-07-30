//! 조그 입력 → 궤적 조합. 툴에서 `robot::motion::Trajectory`를 만든다.

mod draft;
mod kind;
pub mod shooter;

use anyhow::{Context, Result, ensure};
use nalgebra::{Rotation3, Vector3};
use pingpong_bot::Point3;
use pingpong_bot::defaults::{ControlParams, ImpactParams};
use pingpong_bot::estimator::{Impact, Prediction};
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
        Kind::Swing => swing_preview(arm, start, draft).is_ok_and(|p| p.ik_ok),
        _ => true,
    };
}

/// 슈터 공 스윙의 미리보기 정보 — 패널 표시와 `reach_ok`가 함께 쓴다.
pub struct SwingPreview {
    pub prediction: Prediction,
    /// 도달점·법선으로 임팩트 포즈 IK가 풀리는가.
    pub ik_ok: bool,
}

/// 슈터 설정 → 도달 예측 → 임팩트 포즈 IK 가능 여부.
///
/// 예측 자체가 실패하면 `Err` — 그 사유를 패널에 그대로 띄운다.
pub fn swing_preview(arm: &Arm, start: &robot::Pose, draft: &Draft) -> Result<SwingPreview> {
    let prediction = shooter::predict(&draft.shooter, draft.hit_plane_y)?;
    let normal = swing_normal(&prediction, draft)?;
    let ik_ok = arm
        .inverse_pose_with_rail(prediction.impact_position, normal, start)
        .is_ok();
    return Ok(SwingPreview { prediction, ik_ok });
}

/// 라켓 기준 법선 = 입사 반대 방향 + 사용자 기울기.
fn swing_normal(prediction: &Prediction, draft: &Draft) -> Result<Vector3<f64>> {
    let v_in = prediction.incoming_velocity;
    ensure!(v_in.norm() > 1e-3, "입사 속도가 너무 작습니다");
    return tilt_normal(-v_in.normalize(), draft.tilt_pitch_deg, draft.tilt_yaw_deg);
}

fn swing_traj(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
    duration_secs: f64,
    max_delta_deg: f64,
) -> Result<motion::Trajectory> {
    let prediction = shooter::predict(&draft.shooter, draft.hit_plane_y)?;
    let target = prediction.impact_position;
    let v_in = prediction.incoming_velocity;
    let aim_normal = swing_normal(&prediction, draft)?;

    let impact = arm
        .inverse_pose_with_rail(target, aim_normal, start)
        .context("스윙 임팩트 포즈 IK")?;
    let racket = arm
        .forward_kinematics_with_rail(impact.rail_x, &impact.joints)
        .context("임팩트 FK")?;
    let normal = racket.normal.normalize();

    let v_out = Impact::rally_return(target, v_in);
    let e = ImpactParams::default().racket_effective_restitution;
    let v_r = Impact::required_racket_velocity(v_in, v_out, normal, e).context("라켓 속도 역산")?;

    let (rail_impact_vel, joint_impact_vel) = arm
        .velocities_for_racket_velocity(&impact, v_r)
        .context("라켓 속도 → 관절·레일 속도")?;

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
    fn default_shooter_swing_has_a_solution() {
        let built = defaults::robot().expect("robot");
        let start = robot::Pose::new(0.0, built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Swing;
        let preview = swing_preview(&built.arm, &start, &draft).expect("예측은 성공해야 한다");
        assert!(preview.ik_ok, "기본 슈터 공은 IK가 풀려야 한다");
        compose(&built.arm, &start, &draft, 1.0, 90.0).expect("스윙 궤적이 만들어져야 한다");
    }

    #[test]
    fn unreachable_shooter_swing_reports_reason() {
        let built = defaults::robot().expect("robot");
        let start = robot::Pose::new(0.0, built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Swing;
        draft.shooter.pitch_deg = 0.0;
        draft.shooter.height_offset_m = -0.35;
        draft.shooter.speed_mps = 12.0;
        let err = compose(&built.arm, &start, &draft, 1.0, 90.0).unwrap_err();
        assert!(format!("{err:#}").contains("도달"), "{err:#}");
        assert!(!reach_ok(&built.arm, &start, &draft));
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
