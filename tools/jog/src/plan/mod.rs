//! 조그 입력 → 궤적 조합. 툴에서 `robot::motion::Trajectory`를 만든다.

mod draft;
mod kind;
pub mod shooter;

use anyhow::{Context, Result, ensure};
use nalgebra::{Rotation3, Vector3};
use pingpong_bot::Point3;
use pingpong_bot::estimator::Prediction;
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
        Kind::Swing => swing_traj(arm, start, draft, max_delta_deg),
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
        Kind::Swing => swing_preview(arm, start, draft).is_ok(),
        _ => true,
    };
}

/// 슈터 공 스윙의 미리보기 정보 — 패널 표시와 `reach_ok`가 함께 쓴다.
pub struct SwingPreview {
    /// planner가 고른 타점 (접수 창 후보 중 최적).
    pub prediction: Prediction,
}

/// 슈터 설정 → 커밋 시점 예측 묶음 → 시뮬과 같은 planner.
///
/// 접수 평면을 사람이 고르지 않는다 — `plan_best_swing`이 접수 창 후보를 전부
/// 채점해 최적 타점과 궤적을 고른다. 라켓 법선·임팩트 속도도 planner가 푼다.
fn plan_shooter_swing(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
) -> Result<motion::PlannedIntercept> {
    let predictions = shooter::commit_predictions(&draft.shooter)?;
    return motion::physics::plan_best_swing(arm, &predictions, start)
        .map_err(|e| anyhow::anyhow!("{e}"))
        .context("스윙 계획");
}

pub fn swing_preview(arm: &Arm, start: &robot::Pose, draft: &Draft) -> Result<SwingPreview> {
    let planned = plan_shooter_swing(arm, start, draft)?;
    return Ok(SwingPreview {
        prediction: planned.prediction,
    });
}

fn swing_traj(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
    max_delta_deg: f64,
) -> Result<motion::Trajectory> {
    let planned = plan_shooter_swing(arm, start, draft)?;
    ensure_max_delta(&start.joints, &planned.trajectory.end, max_delta_deg)?;
    return Ok(planned.trajectory);
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
        // 시뮬은 랠리 사이 레일 중앙에서 대기한다 — 끝단에서 시작하면 커밋
        // 시간창 안에 레일이 못 간다.
        let rail = built.arm.rail.expect("rail");
        let start = robot::Pose::new(rail.default_x(), built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Swing;
        let preview = swing_preview(&built.arm, &start, &draft).expect("스윙 계획은 성공해야 한다");
        assert!(
            preview.prediction.impact_position.coords.y > 0.0,
            "타점이 로봇 앞이어야 한다"
        );
        compose(&built.arm, &start, &draft, 1.0, 90.0).expect("스윙 궤적이 만들어져야 한다");
    }

    #[test]
    fn unreachable_shooter_swing_reports_reason() {
        let built = defaults::robot().expect("robot");
        let rail = built.arm.rail.expect("rail");
        let start = robot::Pose::new(rail.default_x(), built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Swing;
        draft.shooter.pitch_deg = 0.0;
        draft.shooter.height_offset_m = -0.35;
        draft.shooter.speed_mps = 12.0;
        let err = compose(&built.arm, &start, &draft, 1.0, 90.0).unwrap_err();
        let text = format!("{err:#}");
        assert!(
            text.contains("도달") || text.contains("넘어오지") || text.contains("스윙 계획"),
            "{text}"
        );
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
