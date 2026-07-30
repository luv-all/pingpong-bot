//! 다중 IK 시드 임팩트 후보 평가.

use nalgebra::Vector3;

use crate::defaults;
use crate::error::SwingPlanError;
use crate::estimator::Prediction;
use crate::motion::Impact;
use crate::robot::{self, Arm, Joints};

/// `hint`를 어깨/팔꿈치 한계 구간 중점 기준으로 반사한 대안 시드들을
/// 만든다 — 수치 IK가 같은 목표 자세에 도달하는 다른 관절 조합(다른
/// elbow-up/down류 basin)으로 수렴하도록 시드를 다양화한다. 이 배열의
/// 첫 항목은 항상 원본 `hint` 그대로.
///
/// 근거(2026-07-23): 같은 목표 위치·법선에 도달하는 IK 해가 어떤 관절
/// 조합을 쓰느냐에 따라, 특정 리턴 방향에 대한 자코비안 조작성이 최대
/// 7배 이상 차이 남을 실측 확인 — 시드 하나만 쓰면 우연히 최악
/// 조작성(특이점 근접) 자세로 수렴할 수 있다.
pub(crate) fn candidate_ik_hints(arm: &Arm, hint: &Joints) -> Vec<Joints> {
    let mut hints = vec![hint.clone()];
    let reflect = |joint_index: usize, joints: &Joints| -> Option<Joints> {
        let limit = arm.joint_limit(joint_index)?;
        let mid = (limit.min + limit.max) * 0.5;
        let mut reflected = joints.clone();
        reflected.values[joint_index] =
            (2.0 * mid - joints.values[joint_index]).clamp(limit.min, limit.max);
        return Some(reflected);
    };
    if let Some(shoulder_reflected) = reflect(1, hint) {
        hints.push(shoulder_reflected.clone());
        if let Some(both_reflected) = reflect(2, &shoulder_reflected) {
            hints.push(both_reflected);
        }
    }
    if let Some(elbow_reflected) = reflect(2, hint) {
        hints.push(elbow_reflected);
    }
    return hints;
}

/// 후보 IK 해 하나의 평가 결과 - 목표 방향에 대한 관절속도 조작성 비교용.
pub(crate) struct ImpactCandidate {
    pub(crate) peak_joint_speed_ratio: f64,
    pub(crate) pose: robot::Pose,
    pub(crate) racket_velocity: Vector3<f64>,
    pub(crate) rail_velocity: f64,
    pub(crate) joint_velocities: Vec<f64>,
}

/// 여러 IK 시드를 시도해 목표 리턴 방향에 대해 관절속도 조작성이 가장
/// 좋은(피크 관절속도 비율이 가장 낮은) 해를 고른다 - `inverse_pose_with_rail`
/// 하나만 부르면 첫 수렴 시드에 안주해 우연히 특이점 근접 자세를 고를 수
/// 있다(2026-07-23 실측: 같은 목표를 반사 시드로 재시도하면 관절 조합이
/// 달라져 조작성이 크게 개선될 수 있음을 확인). `plan_swing`/`plan_bang_bang_swing`
/// (내부용, [`solve_impact_target`])과 마운트 위치 튜닝 도구
/// ([`swing_feasibility`], 외부 공개용)가 이 탐색을 공유한다.
pub(crate) fn best_impact_candidate(
    arm: &Arm,
    prediction: &Prediction,
    start: &robot::Pose,
) -> Result<ImpactCandidate, SwingPlanError> {
    let impact_position = prediction.impact_position;
    let v_in = prediction.incoming_velocity;
    let v_out = Impact::rally_return(impact_position, v_in);
    let desired_normal = (v_out - v_in).normalize();

    let base_hint = arm.with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))?;
    let racket_center = crate::Point3::from(
        impact_position.coords
            - desired_normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z),
    );

    let mut best: Option<ImpactCandidate> = None;
    let mut last_error = None;
    for hint in candidate_ik_hints(arm, &base_hint) {
        let solved = match arm.inverse_pose_with_rail(
            racket_center,
            desired_normal,
            &robot::Pose::new(start.rail_x, hint),
        ) {
            Ok(solved) => solved,
            Err(error) => {
                last_error = Some(error);
                continue;
            }
        };
        if crate::robot::collision::table_penetration(arm, solved.rail_x, &solved.joints) > 1e-3 {
            continue;
        }
        let Some(pose) = arm.forward_kinematics_with_rail(solved.rail_x, &solved.joints) else {
            continue;
        };
        let v_r = match Impact::required_racket_velocity(
            v_in,
            v_out,
            pose.normal,
            defaults::ImpactParams::default().racket_effective_restitution,
        ) {
            Ok(v_r) => v_r,
            Err(error) => {
                last_error = Some(error);
                continue;
            }
        };
        // 위치 3제약만의 최소노름 해 - 순간 라켓 방향 고정은 강제하지
        // 않는다(실제 스윙도 접촉 순간 라켓이 계속 회전 중이라 물리적으로
        // 과잉제약이었다, 2026-07-23 실측).
        let (rail_velocity, joint_velocities) =
            match arm.linear_velocities_for_racket_velocity(&solved, v_r) {
                Ok(result) => result,
                Err(error) => {
                    last_error = Some(error);
                    continue;
                }
            };
        let peak_joint_speed_ratio = joint_velocities
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max)
            / arm.max_joint_speed;
        if best
            .as_ref()
            .is_none_or(|candidate| peak_joint_speed_ratio < candidate.peak_joint_speed_ratio)
        {
            best = Some(ImpactCandidate {
                peak_joint_speed_ratio,
                pose: solved,
                racket_velocity: v_r,
                rail_velocity,
                joint_velocities,
            });
        }
    }

    return best.ok_or_else(|| {
        last_error.unwrap_or(SwingPlanError::InverseKinematicsNoSolution {
            target_x: impact_position.coords.x,
            target_y: impact_position.coords.y,
            target_z: impact_position.coords.z,
        })
    });
}
