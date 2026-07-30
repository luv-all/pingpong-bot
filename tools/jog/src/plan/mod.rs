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
        Kind::Swing => anyhow::bail!("스윙은 plan_swing()으로 계획합니다"),
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
        Kind::Swing => true,
        _ => true,
    };
}

/// 슈터 공 스윙 계획 — 코스 추종 이동(필요하면) + 스윙.
///
/// 시뮬은 공이 날아오는 동안 레일·관절을 예측 쪽으로 미리 옮겨두고
/// (`plan_coarse_track`) 커밋 창에서 스윙한다. jog는 그 두 단계를 순서대로
/// 재생할 궤적으로 만든다 — 안 그러면 레일이 먼 대기 위치(예: x=0)에 있을 때
/// 커밋 시간창 안에 도달할 수 없어 모든 해가 실패한다.
#[derive(Debug, Clone)]
pub struct SwingPlan {
    /// planner가 고른 타점.
    pub prediction: Prediction,
    /// 순서대로 재생할 궤적 — `[코스 추종 이동, 스윙]` 또는 `[스윙]`.
    pub segments: Vec<motion::Trajectory>,
    /// 코스 추종으로 옮겨갈 대기 포즈 (이미 가까우면 `None`).
    pub track_pose: Option<robot::Pose>,
}

/// 슈터 설정 → 커밋 시점 예측 → 코스 추종 + 시뮬과 같은 planner.
pub fn plan_swing(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
    track_secs: f64,
    max_delta_deg: f64,
) -> Result<SwingPlan> {
    let predictions = shooter::commit_predictions(&draft.shooter)?;

    // 시뮬의 coarse 추종과 같은 목표. IK가 안 풀리면 현재 포즈에서 바로 스윙.
    let track_pose = motion::physics::plan_coarse_track(arm, &predictions)
        .filter(|pose| pose != start)
        .filter(|pose| ensure_rail_in_range(arm, pose.rail_x).is_ok());

    let swing_start = track_pose.as_ref().unwrap_or(start);
    let planned = motion::physics::plan_best_swing(arm, &predictions, swing_start)
        .map_err(|e| anyhow::anyhow!("{e}"))
        .context("스윙 계획")?;

    let mut segments = Vec::with_capacity(2);
    if let Some(pose) = &track_pose {
        ensure_max_delta(&start.joints, &pose.joints, max_delta_deg).context("코스 추종 이동")?;
        segments.push(move_traj(
            arm,
            start,
            pose.joints.clone(),
            pose.rail_x,
            track_secs,
            max_delta_deg,
        )?);
    }
    ensure_max_delta(&swing_start.joints, &planned.trajectory.end, max_delta_deg)
        .context("스윙")?;
    segments.push(planned.trajectory);

    return Ok(SwingPlan {
        prediction: planned.prediction,
        segments,
        track_pose,
    });
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
        let plan = plan_swing(&built.arm, &start, &draft, 1.0, 90.0).expect("스윙 계획");
        assert!(
            plan.prediction.impact_position.coords.y > 0.0,
            "타점이 로봇 앞이어야 한다"
        );
        assert!(!plan.segments.is_empty());
    }

    /// 레일이 대기 끝단(x=0)이어도 코스 추종 세그먼트가 앞에 붙어 계획된다 —
    /// dry-run boot 포즈에서 모든 해가 실패하던 회귀.
    #[test]
    fn swing_from_rail_home_prepends_coarse_track() {
        let built = defaults::robot().expect("robot");
        let start = robot::Pose::new(0.0, built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Swing;
        let plan = plan_swing(&built.arm, &start, &draft, 1.0, 90.0).expect("스윙 계획");
        assert_eq!(plan.segments.len(), 2, "코스 추종 + 스윙");
        let track = plan.track_pose.expect("코스 추종 포즈");
        assert!(
            (track.rail_x - start.rail_x).abs() > 0.1,
            "레일이 타점 쪽으로 옮겨져야 한다: {}",
            track.rail_x
        );
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
        let err = plan_swing(&built.arm, &start, &draft, 1.0, 90.0).unwrap_err();
        let text = format!("{err:#}");
        assert!(
            text.contains("도달") || text.contains("넘어오지") || text.contains("스윙 계획"),
            "{text}"
        );
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
