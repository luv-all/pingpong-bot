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
    let v_out = Impact::rally_return(prediction.impact_position, prediction.incoming_velocity);
    return best_impact_candidate_for_outgoing(arm, prediction, start, v_out);
}

/// [`best_impact_candidate`]와 같으나 `Impact::rally_return`으로 출사속도를
/// 다시 계산하지 않고 `v_out`을 그대로 받는다 — WP3 진단이 랠리 리턴
/// 목표(`rally_target_y_frac`)를 바꿔 가며 같은 IK/시드 탐색을 재사용한다.
pub(crate) fn best_impact_candidate_for_outgoing(
    arm: &Arm,
    prediction: &Prediction,
    start: &robot::Pose,
    v_out: Vector3<f64>,
) -> Result<ImpactCandidate, SwingPlanError> {
    let impact_position = prediction.impact_position;
    let v_in = prediction.incoming_velocity;
    let desired_normal = (v_out - v_in).normalize();

    let base_hint = arm.with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))?;
    let racket_center = crate::Point3::from(
        impact_position.coords
            - desired_normal
                * (crate::constants::BALL_RADIUS + crate::constants::geometry::RACKET_HALF_Z),
    );

    let mut best: Option<ImpactCandidate> = None;
    let mut last_error = None;
    let try_hint = |hint: Joints,
                    best: &mut Option<ImpactCandidate>,
                    last_error: &mut Option<SwingPlanError>| {
        let solved = match arm.inverse_pose_with_rail(
            racket_center,
            desired_normal,
            &robot::Pose::new(start.rail_x, hint),
        ) {
            Ok(solved) => solved,
            Err(error) => {
                *last_error = Some(error);
                return;
            }
        };
        if crate::robot::collision::table_penetration(arm, solved.rail_x, &solved.joints) > 1e-3 {
            return;
        }
        let Some(pose) = arm.forward_kinematics_with_rail(solved.rail_x, &solved.joints) else {
            return;
        };
        let v_r = match Impact::required_racket_velocity(
            v_in,
            v_out,
            pose.normal,
            defaults::ImpactParams::default().racket_effective_restitution,
        ) {
            Ok(v_r) => v_r,
            Err(error) => {
                *last_error = Some(error);
                return;
            }
        };
        // 위치 3제약만의 최소노름 해 - 순간 라켓 방향 고정은 강제하지
        // 않는다(실제 스윙도 접촉 순간 라켓이 계속 회전 중이라 물리적으로
        // 과잉제약이었다, 2026-07-23 실측).
        let (rail_velocity, joint_velocities) =
            match arm.linear_velocities_for_racket_velocity(&solved, v_r) {
                Ok(result) => result,
                Err(error) => {
                    *last_error = Some(error);
                    return;
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
            *best = Some(ImpactCandidate {
                peak_joint_speed_ratio,
                pose: solved,
                racket_velocity: v_r,
                impact_normal: pose.normal,
                rail_velocity,
                joint_velocities,
            });
        }
    };

    // WP4a(2026-07-30): 1차로 elbow-up 단일 시드(`base_hint`)만 시도한다.
    // 실측(`diag_wp4a_single_vs_multi_seed`, 150개 지오메트리) — WP11
    // (자체운동)이 이미 배선된 뒤로는 반사 시드가 찾는 `r` 개선이 평균
    // Δr=0.0007(최대 0.003)로 사실상 0이고, `base_hint`가 이 arm의 도달
    // 범위 전체에서 항상 elbow-up(q2<0)으로 수렴했다(150/150) — 반사
    // 시드 3개를 매번 도는 IK 비용을 아낄 수 있다. 실패 시에만(단일
    // 시드가 도달불가·테이블관통 등으로 완전히 막힐 때만) 반사 시드로
    // 확장한다 — 계획 원안의 절충안("elbow-up을 1순위로 먼저 시도하고
    // 실패시에만 대안 시드 폴백")대로. `diag_wp4a_single_vs_multi_seed`가
    // 150개 표본에서 이 폴백이 필요했던 사례는 0건이었지만, 그 표본이
    // 못 덮는 기하가 있을 수 있어 안전망으로 남긴다.
    try_hint(base_hint.clone(), &mut best, &mut last_error);
    if best.is_none() {
        for hint in candidate_ik_hints(arm, &base_hint).into_iter().skip(1) {
            try_hint(hint, &mut best, &mut last_error);
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
            for impact_x in [
                table::WIDTH_X * 0.25,
                table::WIDTH_X * 0.5,
                table::WIDTH_X * 0.75,
            ] {
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
                    let Ok(base_hint) = arm
                        .with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))
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
                        let Some(pose) =
                            arm.forward_kinematics_with_rail(solved.rail_x, &solved.joints)
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
                        let r = joint_velocities
                            .iter()
                            .map(|v| v.abs())
                            .fold(0.0_f64, f64::max)
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

    /// 사용자 질문(2026-07-30) 검증용 임시 계측 — impact 속도 IK가 만드는
    /// `joint_velocities` 4개가 한계 대비 **얼마나 고르게 쓰이는가**.
    ///
    /// 질문: "방향만 유지하고 세기를 `min(robot_max, required)`로 클램프하면
    /// 되지 않나?" — 답: 이미 `impact_target_from_candidate`의 `1/r`
    /// 사전축소가 정확히 그 역할이다(v_r을 균일하게 스케일하는 건 선형
    /// 사상이라 "이 방향으로 갈 수 있는 최대치"와 동치). 다만 그 축소가
    /// 기준으로 삼는 `q̇`는 **가중 최소노름**(`linear_velocities_for_
    /// racket_velocity`) 해 하나뿐이다 — 4관절·3속도제약이라 널스페이스가
    /// 1차원 남는데, 최소노름 해가 그 자유도를 안 쓰고 있어서, 한 관절만
    /// 포화되고 나머지는 여유가 남는 상황이면 널스페이스로 여유 관절에
    /// 부하를 옮겨 **같은 방향으로 더 큰 크기**를 낼 여지가 있는지가
    /// 관건이다. 이 계측은 그 여지의 크기를 잰다(구현 전 확인).
    #[test]
    #[ignore = "진단용 계측 — 수치를 stdout으로 뽑는다"]
    fn diag_joint_utilization_at_impact_peak() {
        let robot = crate::defaults::robot().expect("robot");
        let arm = &*robot.arm;
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let start = robot::Pose::new(rail_x, arm.default_joints.clone());
        let window = InterceptWindow::default();

        println!(
            "{:>6} {:>6} {:>7} {:>7} {:>7} {:>7} {:>7} {:>8}",
            "y", "x", "v_in_y", "r", "q0", "q1", "q2", "q3"
        );
        for hit_y in window.hit_planes().into_iter().map(|plane| plane.y) {
            for impact_x in [
                table::WIDTH_X * 0.25,
                table::WIDTH_X * 0.5,
                table::WIDTH_X * 0.75,
            ] {
                for v_in_y in [-5.0_f64, -7.0] {
                    let impact = crate::Point3::new(impact_x, hit_y, table::SURFACE_Z + 0.18);
                    let prediction = Prediction {
                        time_to_impact_secs: 0.30,
                        impact_position: impact,
                        incoming_velocity: Vector3::new(0.0, v_in_y, 0.7),
                    };
                    let Ok(candidate) = best_impact_candidate(arm, &prediction, &start) else {
                        continue;
                    };
                    let util: Vec<f64> = candidate
                        .joint_velocities
                        .iter()
                        .map(|v| v.abs() / arm.max_joint_speed)
                        .collect();
                    println!(
                        "{hit_y:>6.2} {impact_x:>6.2} {v_in_y:>7.1} {:>7.3} {:>7.3} {:>7.3} {:>7.3} {:>8.3}",
                        candidate.peak_joint_speed_ratio,
                        util.first().copied().unwrap_or(f64::NAN),
                        util.get(1).copied().unwrap_or(f64::NAN),
                        util.get(2).copied().unwrap_or(f64::NAN),
                        util.get(3).copied().unwrap_or(f64::NAN),
                    );
                }
            }
        }
    }

    /// WP3 계측 — 고정목표(2점 BVP, y_frac 스윕) vs 최소속도 네트클리어
    /// 공식(사용자 지적, 2026-07-30)이 `peak_joint_speed_ratio`(r)를 얼마나
    /// 낮추는가. WP10이 좁힌 세기 병목("270개 후보 전부 r>2.5, 평균
    /// r=4.114")의 후속 레버 후보였다.
    ///
    /// **결론(기각)**: `min_effort`가 `|v_r|`은 비슷하거나 낮췄지만(1.785→
    /// 1.846, ~3%↑) `r`은 오히려 나빠졌다(2.076→2.721 평균, 3.555→4.822
    /// 최대) — `fixed@0.75`(현 프로덕션 기본값)가 스윕 전체에서 최선이었다.
    /// 원인: `r`을 좌우하는 건 `v_r` 크기가 아니라 그 임팩트 포즈의
    /// 자코비안과 방향이 얼마나 맞는가다(상세: `rally_return_velocity`
    /// doc comment, `docs/wp3-rally-target-distance.md`). `min_effort`는
    /// 프로덕션 기본값에서 제외됐고, 이 스윕은 그 반증 데이터를 재현
    /// 가능하게 남겨 둔다.
    ///
    /// ```text
    /// cargo test --lib diag_wp3_target_distance_sweep -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "진단용 계측 — 수치를 stdout으로 뽑는다"]
    fn diag_wp3_target_distance_sweep() {
        let robot = crate::defaults::robot().expect("robot");
        let arm = &*robot.arm;
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let start = robot::Pose::new(rail_x, arm.default_joints.clone());
        let window = InterceptWindow::default();

        println!(
            "{:>12} {:>6} {:>7} {:>7} {:>7} {:>9} {:>8} {:>8}",
            "scheme", "total", "r_mean", "r>2.5%", "r_max", "|vr|mean", "net_ok%", "ik_ok%"
        );

        // 클로저 하나로 (scheme_label, v_out 계산기)를 스윕한다.
        let fixed = |y_frac: f64| {
            move |impact: crate::Point3| -> Vector3<f64> {
                let cfg = defaults::ImpactParams {
                    rally_target_y_frac: y_frac,
                    ..defaults::ImpactParams::default()
                };
                Impact::rally_return_fixed_point(impact, &cfg)
            }
        };
        let schemes: Vec<(String, Box<dyn Fn(crate::Point3) -> Vector3<f64>>)> = vec![
            ("fixed@0.75".to_string(), Box::new(fixed(0.75))),
            ("fixed@0.65".to_string(), Box::new(fixed(0.65))),
            ("fixed@0.55".to_string(), Box::new(fixed(0.55))),
            (
                "min_effort".to_string(),
                Box::new(|impact| Impact::rally_return(impact, Vector3::zeros())),
            ),
        ];

        for (label, v_out_of) in &schemes {
            let mut r_values: Vec<f64> = Vec::new();
            let mut vr_norms: Vec<f64> = Vec::new();
            let mut net_ok = 0usize;
            let mut ik_ok = 0usize;
            let mut total = 0usize;
            for hit_y in window.hit_planes().into_iter().map(|plane| plane.y) {
                for impact_x in [
                    table::WIDTH_X * 0.25,
                    table::WIDTH_X * 0.5,
                    table::WIDTH_X * 0.75,
                ] {
                    for v_in_y in [-5.0_f64, -7.0] {
                        total += 1;
                        let impact = crate::Point3::new(impact_x, hit_y, table::SURFACE_Z + 0.18);
                        let v_out = v_out_of(impact);
                        if Impact::clears_net(impact, v_out) {
                            net_ok += 1;
                        }
                        let prediction = Prediction {
                            time_to_impact_secs: 0.30,
                            impact_position: impact,
                            incoming_velocity: Vector3::new(0.0, v_in_y, 0.7),
                        };
                        let Ok(candidate) =
                            best_impact_candidate_for_outgoing(arm, &prediction, &start, v_out)
                        else {
                            continue;
                        };
                        ik_ok += 1;
                        r_values.push(candidate.peak_joint_speed_ratio);
                        vr_norms.push(candidate.racket_velocity.norm());
                    }
                }
            }
            let r_mean = r_values.iter().sum::<f64>() / r_values.len().max(1) as f64;
            let r_over = 100.0 * r_values.iter().filter(|&&r| r > 2.5).count() as f64
                / r_values.len().max(1) as f64;
            let r_max = r_values.iter().cloned().fold(0.0_f64, f64::max);
            let vr_mean = vr_norms.iter().sum::<f64>() / vr_norms.len().max(1) as f64;
            println!(
                "{label:>12} {total:>6} {r_mean:>7.3} {r_over:>7.1} {r_max:>7.3} {vr_mean:>9.3} \
                 {:>8.1} {:>8.1}",
                100.0 * net_ok as f64 / total as f64,
                100.0 * ik_ok as f64 / total as f64,
            );
        }
    }

    /// WP4a 계측 — 단일 시드(elbow-up 기준, `base_hint` 그대로) vs 현재
    /// 다중 시드(최대 4개 반사, `candidate_ik_hints`)가 `r`·성공률에
    /// 미치는 영향. WP11(자체운동)이 이미 배선된 상태에서 잰다 —
    /// `best_impact_candidate`가 실제로 부르는 것과 같은 계산 경로.
    ///
    /// `elbow_sign` 열은 `base_hint`(시작 자세 `start.joints`에서 IK가
    /// 수렴한 단일 해)의 q2 부호 — `start.joints`가 항상
    /// `default_joints`(elbow-up, q2<0, WP4a 부호규약 진단 참고)에서
    /// 출발하므로 대부분 음수로 수렴하는지 확인한다.
    ///
    /// ```text
    /// cargo test --lib diag_wp4a_single_vs_multi_seed -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "진단용 계측 — 수치를 stdout으로 뽑는다"]
    fn diag_wp4a_single_vs_multi_seed() {
        let robot = crate::defaults::robot().expect("robot");
        let arm = &*robot.arm;
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let start = robot::Pose::new(rail_x, arm.default_joints.clone());
        let window = InterceptWindow::default();

        let mut single_r: Vec<f64> = Vec::new();
        let mut multi_r: Vec<f64> = Vec::new();
        let mut single_fail_multi_ok = 0usize;
        let mut both_fail = 0usize;
        let mut total = 0usize;
        let mut elbow_up_count = 0usize;
        let mut elbow_down_count = 0usize;

        let score_hints = |hints: Vec<Joints>,
                           racket_center: crate::Point3,
                           desired_normal: Vector3<f64>,
                           v_in: Vector3<f64>,
                           v_out: Vector3<f64>|
         -> Option<f64> {
            let mut best: Option<f64> = None;
            for hint in hints {
                let Ok(solved) = arm.inverse_pose_with_rail(
                    racket_center,
                    desired_normal,
                    &robot::Pose::new(start.rail_x, hint),
                ) else {
                    continue;
                };
                if crate::robot::collision::table_penetration(arm, solved.rail_x, &solved.joints)
                    > 1e-3
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
                let r = joint_velocities
                    .iter()
                    .map(|v| v.abs())
                    .fold(0.0_f64, f64::max)
                    / arm.max_joint_speed;
                if best.is_none_or(|b| r < b) {
                    best = Some(r);
                }
            }
            return best;
        };

        for hit_y in window.hit_planes().into_iter().map(|plane| plane.y) {
            for impact_x in [
                table::WIDTH_X * 0.2,
                table::WIDTH_X * 0.35,
                table::WIDTH_X * 0.5,
                table::WIDTH_X * 0.65,
                table::WIDTH_X * 0.8,
            ] {
                for v_in_y in [-4.0_f64, -5.5, -7.0] {
                    total += 1;
                    let impact = crate::Point3::new(impact_x, hit_y, table::SURFACE_Z + 0.18);
                    let v_in = Vector3::new(0.0, v_in_y, 0.7);
                    let v_out = Impact::rally_return(impact, v_in);
                    let desired_normal = (v_out - v_in).normalize();
                    let Ok(base_hint) = arm
                        .with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))
                    else {
                        continue;
                    };
                    if base_hint.values[2] < 0.0 {
                        elbow_up_count += 1;
                    } else {
                        elbow_down_count += 1;
                    }
                    let racket_center = crate::Point3::from(
                        impact.coords
                            - desired_normal
                                * (crate::constants::BALL_RADIUS
                                    + crate::constants::geometry::RACKET_HALF_Z),
                    );

                    let r_single = score_hints(
                        vec![base_hint.clone()],
                        racket_center,
                        desired_normal,
                        v_in,
                        v_out,
                    );
                    let r_multi = score_hints(
                        candidate_ik_hints(arm, &base_hint),
                        racket_center,
                        desired_normal,
                        v_in,
                        v_out,
                    );

                    match (r_single, r_multi) {
                        (Some(rs), Some(rm)) => {
                            single_r.push(rs);
                            multi_r.push(rm);
                        }
                        (None, Some(_)) => single_fail_multi_ok += 1,
                        (None, None) => both_fail += 1,
                        (Some(_), None) => unreachable!("다중 시드는 단일 시드의 상위집합"),
                    }
                }
            }
        }

        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len().max(1) as f64;
        let over = |v: &[f64]| {
            100.0 * v.iter().filter(|&&r| r > 2.5).count() as f64 / v.len().max(1) as f64
        };
        let max = |v: &[f64]| v.iter().cloned().fold(0.0_f64, f64::max);

        println!(
            "총 {total}개 지오메트리 (both-succeed={}, single만 실패={single_fail_multi_ok}, 둘 다 실패={both_fail})",
            single_r.len()
        );
        println!(
            "base_hint(시작=default_joints) 수렴 부호: elbow-up(q2<0)={elbow_up_count}  elbow-down(q2>=0)={elbow_down_count}"
        );
        println!(
            "{:>10} {:>9} {:>7} {:>9}",
            "scheme", "r_mean", "r>2.5%", "r_max"
        );
        println!(
            "{:>10} {:>9.6} {:>7.1} {:>9.6}",
            "single",
            mean(&single_r),
            over(&single_r),
            max(&single_r)
        );
        println!(
            "{:>10} {:>9.6} {:>7.1} {:>9.6}",
            "multi",
            mean(&multi_r),
            over(&multi_r),
            max(&multi_r)
        );
        let diffs: Vec<f64> = single_r
            .iter()
            .zip(multi_r.iter())
            .map(|(s, m)| s - m)
            .collect();
        let improved = diffs.iter().filter(|d| **d > 1e-6).count();
        let max_improvement = diffs.iter().cloned().fold(0.0_f64, f64::max);
        let mean_improvement_when_improved = if improved > 0 {
            diffs.iter().filter(|d| **d > 1e-6).sum::<f64>() / improved as f64
        } else {
            0.0
        };
        println!(
            "다중 시드가 단일보다 더 낮은 r을 찾은 지오메트리: {improved}/{} \
             (개선됐을 때 평균 Δr={mean_improvement_when_improved:.6}, 최대 Δr={max_improvement:.6})",
            single_r.len()
        );
    }

    /// WP4a 계측 — 단일시드 우선(현 기본값) vs 항상 다중시드 4개를 다 도는
    /// 경로의 벽시계 비용. `best_impact_candidate`(프로덕션 진입점, 단일
    /// 우선+실패시 폴백)와, `candidate_ik_hints`의 반사 3개를 강제로 항상
    /// 도는 비교군을 같은 150개 지오메트리에 반복 호출해 잰다.
    ///
    /// ```text
    /// cargo test --lib diag_wp4a_seed_search_cost -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "진단용 계측 — 수치를 stdout으로 뽑는다"]
    fn diag_wp4a_seed_search_cost() {
        let robot = crate::defaults::robot().expect("robot");
        let arm = &*robot.arm;
        let rail_x = arm.rail.as_ref().map(|r| r.default_x()).unwrap_or(0.0);
        let start = robot::Pose::new(rail_x, arm.default_joints.clone());
        let window = InterceptWindow::default();

        let predictions: Vec<Prediction> = window
            .hit_planes()
            .into_iter()
            .flat_map(|plane| {
                [
                    table::WIDTH_X * 0.2,
                    table::WIDTH_X * 0.35,
                    table::WIDTH_X * 0.5,
                    table::WIDTH_X * 0.65,
                    table::WIDTH_X * 0.8,
                ]
                .into_iter()
                .flat_map(move |x| {
                    [-4.0_f64, -5.5, -7.0]
                        .into_iter()
                        .map(move |v_in_y| Prediction {
                            time_to_impact_secs: 0.30,
                            impact_position: crate::Point3::new(
                                x,
                                plane.y,
                                table::SURFACE_Z + 0.18,
                            ),
                            incoming_velocity: Vector3::new(0.0, v_in_y, 0.7),
                        })
                })
            })
            .collect();

        const REPEATS: usize = 20;

        let t0 = std::time::Instant::now();
        for _ in 0..REPEATS {
            for prediction in &predictions {
                let _ = best_impact_candidate(arm, prediction, &start);
            }
        }
        let single_elapsed = t0.elapsed();

        // 강제 다중시드 비교군 — candidate_ik_hints 전체를 항상 돈다
        // (best_impact_candidate_for_outgoing 이전 구현과 동등).
        let force_multi = |prediction: &Prediction| {
            let v_out =
                Impact::rally_return(prediction.impact_position, prediction.incoming_velocity);
            let v_in = prediction.incoming_velocity;
            let desired_normal = (v_out - v_in).normalize();
            let Ok(base_hint) =
                arm.with_wrist_open(&start.joints, Arm::wrist_open_for_return(v_out - v_in))
            else {
                return;
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
                let _ =
                    crate::robot::collision::table_penetration(arm, solved.rail_x, &solved.joints);
                if let Some(pose) = arm.forward_kinematics_with_rail(solved.rail_x, &solved.joints)
                    && let Ok(v_r) = Impact::required_racket_velocity(
                        v_in,
                        v_out,
                        pose.normal,
                        defaults::ImpactParams::default().racket_effective_restitution,
                    )
                {
                    let _ = arm.linear_velocities_for_racket_velocity(&solved, v_r);
                }
            }
        };
        let t1 = std::time::Instant::now();
        for _ in 0..REPEATS {
            for prediction in &predictions {
                force_multi(prediction);
            }
        }
        let multi_elapsed = t1.elapsed();

        let calls = REPEATS * predictions.len();
        println!(
            "{calls}회 호출: single-first={:.3}ms/call  force-multi={:.3}ms/call  비율={:.2}x",
            single_elapsed.as_secs_f64() * 1000.0 / calls as f64,
            multi_elapsed.as_secs_f64() * 1000.0 / calls as f64,
            multi_elapsed.as_secs_f64() / single_elapsed.as_secs_f64().max(1e-9)
        );
    }
}
