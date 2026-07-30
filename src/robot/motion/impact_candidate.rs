//! 다중 IK 시드 임팩트 후보 평가.

use nalgebra::Vector3;

use crate::defaults;
use crate::error::SwingPlanError;
use crate::estimator::Impact;
use crate::estimator::Prediction;
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
    /// IK 해가 실제로 만드는 라켓 면 법선 — `racket_velocity`의 법선 성분
    /// (`v_r·n`)이 리턴 세기를 지배하므로 WP2b 복합 랭킹이 이걸 쓴다.
    pub(crate) impact_normal: Vector3<f64>,
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
///
/// **WP2b(2026-07-30): 이 시드 랭킹은 `peak_joint_speed_ratio` 단독을
/// 유지한다** — 타점 간 랭킹([`plan_best_swing`])만 복합 점수로 바꿨다.
/// 근거는 `diag_wp2b_ik_seed_spread`(아래 `tests`) 실측: **같은 타점**의
/// 시드들은 필요 라켓속도 `|v_r|`이 서로 최대 **0.026%**밖에 다르지 않다
/// (`v_r`은 타점 기하가 정하고, 시드가 바꾸는 건 그걸 내는 관절 조합뿐이라
/// IK 수렴 오차만큼만 갈린다). 달성 세기 ≈ `|v_r| × min(1, 1/r)`에서
/// `|v_r|`이 상수면 `r` 최소화가 곧 세기 최대화다 — 여기서 복합 점수는
/// 같은 순서를 더 비싸게 계산하는 것에 불과하다.
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
                impact_normal: pose.normal,
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

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::constants::table;
    use crate::estimator::Prediction;
    use crate::robot::motion::InterceptWindow;

    /// WP2b 계측 — **같은 타점**의 IK 시드끼리 필요 라켓속도 `v_r`이 얼마나
    /// 다른가.
    ///
    /// 시드 간 랭킹을 `peak_joint_speed_ratio` 단독(현재)에서 복합으로 바꿔야
    /// 하는지 판단하는 근거다. 달성 세기 ≈ `|v_r| × min(1, 1/r)` 이므로,
    /// 같은 타점의 시드들이 사실상 같은 `|v_r|`을 요구한다면 `r` 최소화가
    /// 곧 세기 최대화이고 시드 랭킹은 바꿀 필요가 없다.
    ///
    /// ```text
    /// cargo test --lib diag_wp2b_ik_seed_spread -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "진단용 계측 — 수치를 stdout으로 뽑는다"]
    fn diag_wp2b_ik_seed_spread() {
        let robot = crate::defaults::robot().expect("robot");
        let arm = &*robot.arm;
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let start = robot::Pose::new(rail_x, arm.default_joints.clone());
        let window = InterceptWindow::default();

        println!(
            "{:>6} {:>6} {:>7} {:>5} {:>8} {:>8} {:>8} {:>9} {:>9}",
            "y", "x", "v_in.y", "seeds", "r_min", "r_max", "|v_r|min", "|v_r|max", "spread%"
        );
        let mut worst_spread = 0.0_f64;
        for hit_y in window.hit_planes().into_iter().map(|plane| plane.y) {
            for impact_x in [table::WIDTH_X * 0.25, table::WIDTH_X * 0.5, table::WIDTH_X * 0.75] {
                for v_in_y in [-5.0_f64, -7.0] {
                    let prediction = Prediction {
                        time_to_impact_secs: 0.30,
                        impact_position: crate::Point3::new(
                            impact_x,
                            hit_y,
                            table::SURFACE_Z + 0.18,
                        ),
                        incoming_velocity: Vector3::new(0.0, v_in_y, 0.7),
                    };
                    let mut rows: Vec<(f64, f64)> = Vec::new();
                    // `best_impact_candidate`의 시드 루프를 그대로 재현해
                    // **모든** 시드의 (r, |v_r|)를 남긴다.
                    let v_in = prediction.incoming_velocity;
                    let v_out = Impact::rally_return(prediction.impact_position, v_in);
                    let desired_normal = (v_out - v_in).normalize();
                    let Ok(base_hint) =
                        arm.with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))
                    else {
                        continue;
                    };
                    let racket_center = crate::Point3::from(
                        prediction.impact_position.coords
                            - desired_normal
                                * (crate::constants::BALL_RADIUS
                                    + crate::constants::geometry::RACKET_HALF_Z),
                    );
                    for hint in candidate_ik_hints(arm, &base_hint) {
                        let Ok(solved) = arm.inverse_pose_with_rail(
                            racket_center,
                            desired_normal,
                            &robot::Pose::new(start.rail_x, hint),
                        ) else {
                            continue;
                        };
                        if crate::robot::collision::table_penetration(
                            arm,
                            solved.rail_x,
                            &solved.joints,
                        ) > 1e-3
                        {
                            continue;
                        }
                        let Some(pose) = arm.forward_kinematics_with_rail(solved.rail_x, &solved.joints)
                        else {
                            continue;
                        };
                        let Ok(v_r) = Impact::required_racket_velocity(
                            v_in,
                            v_out,
                            pose.normal,
                            defaults::ImpactParams::default().racket_effective_restitution,
                        ) else {
                            continue;
                        };
                        let Ok((_, joint_velocities)) =
                            arm.linear_velocities_for_racket_velocity(&solved, v_r)
                        else {
                            continue;
                        };
                        let r = joint_velocities.iter().map(|v| v.abs()).fold(0.0_f64, f64::max)
                            / arm.max_joint_speed;
                        rows.push((r, v_r.norm()));
                    }
                    if rows.len() < 2 {
                        continue;
                    }
                    let r_min = rows.iter().map(|x| x.0).fold(f64::INFINITY, f64::min);
                    let r_max = rows.iter().map(|x| x.0).fold(0.0, f64::max);
                    let m_min = rows.iter().map(|x| x.1).fold(f64::INFINITY, f64::min);
                    let m_max = rows.iter().map(|x| x.1).fold(0.0, f64::max);
                    let spread = 100.0 * (m_max - m_min) / m_max.max(1e-9);
                    worst_spread = worst_spread.max(spread);
                    println!(
                        "{hit_y:>6.2} {impact_x:>6.2} {v_in_y:>7.1} {:>5} {r_min:>8.3} {r_max:>8.3} \
                         {m_min:>8.3} {m_max:>9.3} {spread:>9.3}",
                        rows.len()
                    );
                }
            }
        }
        println!("\n같은 타점 내 시드 간 |v_r| 최대 상대 산포 = {worst_spread:.4}%");
    }
}
