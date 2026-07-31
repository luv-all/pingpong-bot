//! IK 자기일관성 진단 — "해 없음"이 진짜 도달범위 밖인지, 솔버가 못 찾는 것인지 가른다.
//!
//! FK로 만든 자세는 **해가 존재함이 정의상 보장된다** (그 자세를 만든 관절각이 곧 해다).
//! 그러므로 여기서의 실패는 전부 솔버 책임이다. 실기 로그의 "역기구학 해 없음"이
//! 도달범위 문제라는 주장은 이 테스트를 통과한 뒤에만 신뢰할 수 있다.
//!
//! ```bash
//! cargo test --test diag_ik_roundtrip -- --nocapture
//! ```

use nalgebra::Vector3;
use pingpong_bot::robot::{IkSearch, Joints, Pose};

/// 관절 한계 안에서 격자 샘플링 — 한계 근처는 피한다(한계 자체가 해를 자르면 FK도 못 만든다).
fn joint_grid(arm: &pingpong_bot::robot::Arm, steps: usize) -> Vec<Joints> {
    let mut per_joint = Vec::new();
    for index in 0..arm.joint_count() {
        // `None`은 URDF continuous 관절 — 한계가 없으니 ±90°만 훑는다.
        let (min, max) = arm.joint_limit(index).map_or(
            (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
            |l| (l.min, l.max),
        );
        // 한계에서 10% 안쪽으로만 — 경계 위 해는 클램프와 구분이 안 된다.
        let lo = min + (max - min) * 0.1;
        let hi = max - (max - min) * 0.1;
        let values: Vec<f64> = (0..steps)
            .map(|s| lo + (hi - lo) * (s as f64) / ((steps - 1).max(1) as f64))
            .collect();
        per_joint.push(values);
    }

    let mut out = vec![Vec::new()];
    for values in &per_joint {
        let mut next = Vec::new();
        for prefix in &out {
            for value in values {
                let mut extended = prefix.clone();
                extended.push(*value);
                next.push(extended);
            }
        }
        out = next;
    }
    return out.into_iter().map(|v| Joints::from_slice(&v)).collect();
}

#[test]
fn ik_recovers_poses_it_generated_itself() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");

    // 레일 위치 3곳 × 관절 격자 — 각 자세를 FK로 만들고 그 자세를 IK에 되먹인다.
    let rail_xs = [
        rail.x_min + (rail.x_max - rail.x_min) * 0.25,
        rail.default_x(),
        rail.x_min + (rail.x_max - rail.x_min) * 0.75,
    ];
    let grid = joint_grid(&arm, 4);
    let hint = Pose::new(rail.default_x(), arm.default_joints.clone());

    let mut total = 0usize;
    let mut solved = 0usize;
    let mut worst_position = 0.0_f64;
    let mut worst_normal = 0.0_f64;
    let mut failures: Vec<(f64, Joints)> = Vec::new();

    for rail_x in rail_xs {
        for joints in &grid {
            let Some(target) = arm.forward_kinematics_with_rail(rail_x, joints) else {
                continue; // FK 자체가 불가능한 조합은 IK 책임이 아니다.
            };
            total += 1;

            match arm.inverse_pose_with_rail(
                target.position,
                target.normal,
                &hint,
                IkSearch::Global,
            ) {
                Ok(pose) => {
                    solved += 1;
                    let actual = arm
                        .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
                        .expect("solved FK");
                    worst_position = worst_position
                        .max((actual.position.coords - target.position.coords).norm());
                    worst_normal = worst_normal.max((actual.normal - target.normal).norm());
                }
                Err(_) => failures.push((rail_x, joints.clone())),
            }
        }
    }

    let rate = 100.0 * (solved as f64) / (total as f64);
    println!("\n=== FK→IK 라운드트립 (해가 존재함이 보장된 표적) ===");
    println!("표적 {total}개 · 성공 {solved}개 ({rate:.1}%)");
    println!("성공 케이스 최대 잔차: 위치 {worst_position:.6} m · 법선 {worst_normal:.6}");

    if !failures.is_empty() {
        println!("\n실패 샘플 (최대 8개):");
        for (rail_x, joints) in failures.iter().take(8) {
            let target = arm
                .forward_kinematics_with_rail(*rail_x, joints)
                .expect("FK");
            let values: Vec<String> = joints.values.iter().map(|v| format!("{v:.2}")).collect();
            println!(
                "  rail={rail_x:.2} joints=[{}] → pos=({:.3}, {:.3}, {:.3}) normal=({:.2}, {:.2}, {:.2})",
                values.join(", "),
                target.position.coords.x,
                target.position.coords.y,
                target.position.coords.z,
                target.normal.x,
                target.normal.y,
                target.normal.z,
            );
        }
    }

    assert!(
        rate > 95.0,
        "FK가 만든 자세를 IK가 {rate:.1}%만 복원한다 — 도달범위가 아니라 솔버 문제다"
    );
}

/// 실패 원인을 가른다: **지역해(seed)** 인가 **수렴/허용오차** 인가?
///
/// 정답에서 조금씩 떨어뜨린 힌트로 같은 표적을 푼다. 힌트가 가까워질수록 성공률이
/// 급등하면 seed 문제(전역 탐색 부족)고, 가까워도 그대로면 수렴·허용오차 문제다.
#[test]
fn ik_failure_is_seed_dependent_or_convergence_dependent() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let grid = joint_grid(&arm, 4);
    let rail_x = rail.default_x();

    println!("\n=== 힌트 거리별 성공률 (같은 표적 집합) ===");
    for perturb in [0.0, 0.05, 0.2, 0.5, 1.0] {
        let mut total = 0usize;
        let mut solved = 0usize;
        for joints in &grid {
            let Some(target) = arm.forward_kinematics_with_rail(rail_x, joints) else {
                continue;
            };
            total += 1;
            // 결정론적 교란 — 관절마다 부호를 번갈아 정답에서 떨어뜨린다.
            let hinted: Vec<f64> = joints
                .values
                .iter()
                .enumerate()
                .map(|(i, v)| v + if i % 2 == 0 { perturb } else { -perturb })
                .collect();
            let hint = Pose::new(rail_x, Joints::from_slice(&hinted));
            if arm
                .inverse_pose_with_rail(target.position, target.normal, &hint, IkSearch::Global)
                .is_ok()
            {
                solved += 1;
            }
        }
        let rate = 100.0 * (solved as f64) / (total as f64);
        println!("  힌트 오차 {perturb:.2} rad → {solved}/{total} ({rate:.1}%)");
    }
}

/// 힌트를 정답으로 주면 0회 반복으로 즉시 수렴해야 한다. 실패하면 수렴이 아니라
/// **허용오차 자체**가 도달 불가능하다는 뜻이다.
#[test]
fn ik_accepts_the_exact_solution_as_its_own_hint() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let grid = joint_grid(&arm, 3);

    let mut total = 0usize;
    let mut solved = 0usize;
    for joints in &grid {
        let rail_x = rail.default_x();
        let Some(target) = arm.forward_kinematics_with_rail(rail_x, joints) else {
            continue;
        };
        total += 1;
        let exact = Pose::new(rail_x, joints.clone());
        if arm
            .inverse_pose_with_rail(target.position, target.normal, &exact, IkSearch::Global)
            .is_ok()
        {
            solved += 1;
        }
    }

    println!("\n=== 정답을 힌트로 준 경우 ===");
    println!("표적 {total}개 · 성공 {solved}개");
    assert_eq!(
        solved, total,
        "정답을 힌트로 줬는데도 실패한다 — 허용오차가 FK 정밀도보다 빡빡하다"
    );
}

/// 퍼뜨린 시드로 멀티스타트하면 성공률이 어디까지 오르는가 — 고칠 여지의 상한.
fn multistart_seeds(arm: &pingpong_bot::robot::Arm, count: usize) -> Vec<Joints> {
    // 결정론적 저불일치 수열 (황금비) — 관절 범위를 고르게 덮는다.
    const GOLDEN: f64 = 0.618_033_988_749_895;
    let mut seeds = vec![arm.default_joints.clone()];
    for s in 1..count {
        let values: Vec<f64> = (0..arm.joint_count())
            .map(|index| {
                let (min, max) = arm.joint_limit(index).map_or(
                    (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
                    |l| (l.min, l.max),
                );
                let frac = ((s as f64) * GOLDEN * ((index + 1) as f64)).fract();
                min + (max - min) * frac
            })
            .collect();
        seeds.push(Joints::from_slice(&values));
    }
    return seeds;
}

#[test]
fn multistart_recovers_what_the_single_seed_solver_misses() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let grid = joint_grid(&arm, 4);
    let rail_x = rail.default_x();

    println!("\n=== 시드 개수별 성공률 (FK가 만든 표적) ===");
    for count in [1, 2, 4, 8, 16, 32] {
        let seeds = multistart_seeds(&arm, count);
        let mut total = 0usize;
        let mut solved = 0usize;
        for joints in &grid {
            let Some(target) = arm.forward_kinematics_with_rail(rail_x, joints) else {
                continue;
            };
            total += 1;
            let hit = seeds.iter().any(|seed| {
                arm.inverse_pose_with_rail(
                    target.position,
                    target.normal,
                    &Pose::new(rail_x, seed.clone()),
                    IkSearch::Local,
                )
                .is_ok()
            });
            if hit {
                solved += 1;
            }
        }
        let rate = 100.0 * (solved as f64) / (total as f64);
        println!("  시드 {count:2}개 → {solved}/{total} ({rate:.1}%)");
    }
}

/// 이산 분기(팔꿈치 위/아래 등)를 노린 시드 — 관절 1·2·3을 한계 중앙 기준으로 반사한
/// 모든 부호 조합. 무작위 시드보다 적은 개수로 같은 커버리지를 내는지 비교한다.
fn branch_seeds(arm: &pingpong_bot::robot::Arm, base: &Joints) -> Vec<Joints> {
    let reflect = |joints: &Joints, index: usize| -> Joints {
        let mut out = joints.clone();
        if let Some(limit) = arm.joint_limit(index) {
            let mid = (limit.min + limit.max) * 0.5;
            out.values[index] = (2.0 * mid - joints.values[index]).clamp(limit.min, limit.max);
        } else {
            out.values[index] = -joints.values[index];
        }
        return out;
    };
    let mut seeds = Vec::new();
    for mask in 0..8u8 {
        let mut seed = base.clone();
        for (bit, joint) in [(0u8, 1usize), (1, 2), (2, 3)] {
            if mask & (1 << bit) != 0 {
                seed = reflect(&seed, joint);
            }
        }
        seeds.push(seed);
    }
    return seeds;
}

#[test]
fn branch_seeds_versus_random_seeds() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let grid = joint_grid(&arm, 4);
    let rail_x = rail.default_x();

    let try_seeds = |seeds: &[Joints]| -> (usize, usize, f64) {
        let start = std::time::Instant::now();
        let mut total = 0usize;
        let mut solved = 0usize;
        for joints in &grid {
            let Some(target) = arm.forward_kinematics_with_rail(rail_x, joints) else {
                continue;
            };
            total += 1;
            if seeds.iter().any(|seed| {
                arm.inverse_pose_with_rail(
                    target.position,
                    target.normal,
                    &Pose::new(rail_x, seed.clone()),
                    IkSearch::Local,
                )
                .is_ok()
            }) {
                solved += 1;
            }
        }
        return (
            solved,
            total,
            start.elapsed().as_secs_f64() * 1e3 / (total as f64),
        );
    };

    println!("\n=== 분기 시드 vs 무작위 시드 ===");
    let branch = branch_seeds(&arm, &arm.default_joints);
    let (s, t, ms) = try_seeds(&branch);
    println!(
        "  분기 시드 {}개 → {s}/{t} ({:.1}%) · 표적당 {ms:.3} ms",
        branch.len(),
        100.0 * s as f64 / t as f64
    );
    for count in [8, 16] {
        let seeds = multistart_seeds(&arm, count);
        let (s, t, ms) = try_seeds(&seeds);
        println!(
            "  무작위 시드 {count:2}개 → {s}/{t} ({:.1}%) · 표적당 {ms:.3} ms",
            100.0 * s as f64 / t as f64
        );
    }
    // 둘을 이어붙이면? (분기 우선 → 실패 시 무작위 보강)
    let mut combined = branch.clone();
    combined.extend(multistart_seeds(&arm, 8));
    let (s, t, ms) = try_seeds(&combined);
    println!(
        "  분기+무작위 {}개 → {s}/{t} ({:.1}%) · 표적당 {ms:.3} ms",
        combined.len(),
        100.0 * s as f64 / t as f64
    );
}

/// 현행 호출 1회의 실제 비용 — 성공/실패를 나눠 잰다. 실패가 비용을 지배하는지 확인.
#[test]
fn baseline_cost_of_one_ik_call() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let grid = joint_grid(&arm, 4);
    let rail_x = rail.default_x();
    let hint = Pose::new(rail_x, arm.default_joints.clone());

    let mut ok_secs = 0.0_f64;
    let mut ok_count = 0usize;
    let mut fail_secs = 0.0_f64;
    let mut fail_count = 0usize;

    for joints in &grid {
        let Some(target) = arm.forward_kinematics_with_rail(rail_x, joints) else {
            continue;
        };
        let start = std::time::Instant::now();
        let result =
            arm.inverse_pose_with_rail(target.position, target.normal, &hint, IkSearch::Global);
        let elapsed = start.elapsed().as_secs_f64();
        if result.is_ok() {
            ok_secs += elapsed;
            ok_count += 1;
        } else {
            fail_secs += elapsed;
            fail_count += 1;
        }
    }

    println!("\n=== 현행 inverse_pose_with_rail 1회 비용 ===");
    println!(
        "  성공 {ok_count}회 · 평균 {:.3} ms",
        1e3 * ok_secs / (ok_count.max(1) as f64)
    );
    println!(
        "  실패 {fail_count}회 · 평균 {:.3} ms",
        1e3 * fail_secs / (fail_count.max(1) as f64)
    );
    println!(
        "  실패가 전체 시간의 {:.0}%",
        100.0 * fail_secs / (ok_secs + fail_secs)
    );
}

/// 실기에서 실제로 요구되는 표적(임팩트 지점 + 리턴 법선)에 가까운 조건.
/// 위 두 테스트가 통과하는데 이게 실패하면, 그때는 진짜 도달범위 문제다.
#[test]
fn ik_reports_reach_for_impact_like_targets() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let hint = Pose::new(rail.default_x(), arm.default_joints.clone());

    // 로봇 코트 쪽 임팩트 평면을 훑는다.
    let mut total = 0usize;
    let mut solved = 0usize;
    let mut unreachable: Vec<(f64, f64, f64)> = Vec::new();
    for x_step in 0..7 {
        for z_step in 0..5 {
            let x = rail.x_min + (rail.x_max - rail.x_min) * (x_step as f64) / 6.0;
            let z = 0.75 + 0.10 * (z_step as f64);
            let y = 0.35;
            let target = nalgebra::Point3::new(x, y, z);
            // 네트 너머로 되돌리는 법선 (−y 방향에서 온 공을 +y로).
            let normal = Vector3::new(0.0, 1.0, 0.15).normalize();
            total += 1;
            if arm
                .inverse_pose_with_rail(target, normal, &hint, IkSearch::Global)
                .is_ok()
            {
                solved += 1;
            } else {
                unreachable.push((x, y, z));
            }
        }
    }

    let rate = 100.0 * (solved as f64) / (total as f64);
    println!("\n=== 임팩트 유사 표적 (위치 + 리턴 법선) ===");
    println!("표적 {total}개 · 도달 {solved}개 ({rate:.1}%)");
    if !unreachable.is_empty() {
        println!("도달 불가 (최대 10개):");
        for (x, y, z) in unreachable.iter().take(10) {
            println!("  ({x:.2}, {y:.2}, {z:.2})");
        }
    }
}

/// 진짜 도달 불가 표적의 최악 비용 — 시드가 늘었으니 실패 경로가 얼마나 비싸졌는지.
#[test]
fn cost_of_a_genuinely_unreachable_target() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let hint = Pose::new(rail.default_x(), arm.default_joints.clone());
    let far = nalgebra::Point3::new(0.7, 5.0, 3.0); // 작업공간에서 명백히 벗어남
    let normal = Vector3::new(0.0, 1.0, 0.0);

    let start = std::time::Instant::now();
    const RUNS: usize = 50;
    for _ in 0..RUNS {
        assert!(
            arm.inverse_pose_with_rail(far, normal, &hint, IkSearch::Global)
                .is_err()
        );
    }
    let ms = start.elapsed().as_secs_f64() * 1e3 / (RUNS as f64);
    println!("\n=== 도달 불가 표적 1회 비용: {ms:.3} ms ===");
}

/// 도달 경계는 **필요조건**이어야 한다 — FK가 실제로 만들 수 있는 자세를 하나라도
/// 자르면 조용히 해를 버리는 것이다. 링크 길이 합 기반 경계를 전 작업공간에서 검증한다.
#[test]
fn reach_bound_never_rejects_a_reachable_pose() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let max_reach = arm.link_lengths.iter().sum::<f64>()
        + pingpong_bot::constants::geometry::RACKET_HANDLE_LENGTH
        + pingpong_bot::constants::geometry::RACKET_HALF_X;

    let grid = joint_grid(&arm, 5);
    let mut worst = 0.0_f64;
    let mut checked = 0usize;
    for rail_x in [rail.x_min, rail.default_x(), rail.x_max] {
        for joints in &grid {
            let Some(pose) = arm.forward_kinematics_with_rail(rail_x, joints) else {
                continue;
            };
            checked += 1;
            let dx = pose.position.coords.x - rail.clamp_x(pose.position.coords.x);
            let dy = pose.position.coords.y - rail.mount_y;
            let dz = pose.position.coords.z - rail.mount_z;
            worst = worst.max((dx * dx + dy * dy + dz * dz).sqrt());
        }
    }

    println!("\n=== 도달 경계 검증 ===");
    println!("  FK 실측 최대거리 {worst:.4} m · 경계 {max_reach:.4} m · 표본 {checked}개");
    assert!(
        worst <= max_reach,
        "경계 {max_reach:.4} m가 실제 도달거리 {worst:.4} m보다 작다 — 해를 잘라낸다"
    );
}

/// 실기 실패 표적(fly_05·fly_07)이 **위치**가 안 닿는 건지 **자세(법선)** 가 안 나오는
/// 건지 가른다. 에러 메시지는 "위치가 도달 범위 밖"이라 말하지만 둘은 다른 문제다.
#[test]
fn separate_position_reach_from_orientation_feasibility() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let hint = Pose::new(rail.default_x(), arm.default_joints.clone());
    let max_reach = arm.link_lengths.iter().sum::<f64>()
        + pingpong_bot::constants::geometry::RACKET_HANDLE_LENGTH
        + pingpong_bot::constants::geometry::RACKET_HALF_X;

    println!("\n=== 실기 실패 표적 분해 ===");
    println!(
        "레일 x 범위 [{:.2}, {:.2}] · 마운트 y={:.2} z={:.2} · 최대 도달 {max_reach:.3} m",
        rail.x_min, rail.x_max, rail.mount_y, rail.mount_z
    );
    for (x, y, z) in [
        (1.590, 0.328, 0.993), // fly_05
        (1.619, 0.328, 0.990), // fly_05
        (1.658, 0.328, 0.956), // fly_07
        (1.613, 0.328, 0.970), // fly_07
        (0.640, 0.230, 1.010), // fly_04 — 커밋에 성공한 표적 (대조군)
    ] {
        let target = nalgebra::Point3::new(x, y, z);
        let rail_x = rail.clamp_x(x);
        let dx = x - rail_x;
        let dy = y - rail.mount_y;
        let dz = z - rail.mount_z;
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();

        let position_only = arm
            .inverse_kinematics_with_rail(&rail, rail_x, target, Some(&arm.default_joints))
            .is_ok();
        // 리턴 법선은 대략 +y로 되돌리는 방향.
        let normal = Vector3::new(0.0, 1.0, 0.2).normalize();
        let pose_ok = arm
            .inverse_pose_with_rail(target, normal, &hint, IkSearch::Global)
            .is_ok();

        println!(
            "  ({x:.3}, {y:.3}, {z:.3}) · 레일밖 {dx:+.3} m · 마운트거리 {distance:.3} m \
             → 위치IK {} · 자세IK {}",
            if position_only { "성공" } else { "실패" },
            if pose_ok { "성공" } else { "실패" },
        );
    }
}

/// 요구 법선을 못 맞출 때 **얼마나** 어긋나는지 잰다.
///
/// 팔 4축 + 레일 1축에 위치 구속은 3개뿐이라 한 위치를 만드는 자세는 2차원 족(族)을
/// 이룬다. 그 족을 훑어 달성 가능한 법선 중 요구 법선에 가장 가까운 것을 찾는다.
/// 어긋남이 작으면 "그냥 치게" 하는 것이 옳고, 90°에 가까우면 라켓 모서리로 맞는
/// 것이라 타격이 아니다 — 그 경계를 수치로 봐야 정할 수 있다.
#[test]
fn how_far_off_is_the_achievable_racket_normal() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");

    println!("\n=== 달성 가능한 최선 법선과 요구 법선의 각도차 ===");
    for (label, x, y, z, nx, ny, nz) in [
        ("fly_05", 1.585, 0.328, 0.993, -0.37, 0.89, 0.26),
        ("fly_07", 1.652, 0.328, 0.955, -0.37, 0.89, 0.26),
        ("fly_04(대조군)", 0.640, 0.230, 1.010, 0.0, 1.0, 0.2),
    ] {
        let target = nalgebra::Point3::new(x, y, z);
        let desired = Vector3::new(nx, ny, nz).normalize();
        let rail_x = rail.clamp_x(x);

        // 자세 족을 시드로 훑는다 — 위치만 구속하고 달성된 법선을 모은다.
        let mut best_deg = f64::INFINITY;
        let mut best_position_error = f64::INFINITY;
        let mut reached = 0usize;
        for seed in multistart_seeds(&arm, 64) {
            let Ok(joints) = arm.inverse_kinematics_with_rail(&rail, rail_x, target, Some(&seed))
            else {
                continue;
            };
            let Some(pose) = arm.forward_kinematics_with_rail(rail_x, &joints) else {
                continue;
            };
            let position_error = (pose.position.coords - target.coords).norm();
            if position_error > 2e-3 {
                continue; // 위치를 못 맞춘 해는 타격이 아니다.
            }
            reached += 1;
            let cos = pose.normal.dot(&desired).clamp(-1.0, 1.0);
            let deg = cos.acos().to_degrees();
            if deg < best_deg {
                best_deg = deg;
                best_position_error = position_error;
            }
        }

        if reached == 0 {
            println!("  {label:14} → 위치 자체가 도달 불가");
        } else {
            println!(
                "  {label:14} → 위치해 {reached}개 · 최선 법선 어긋남 {best_deg:.1}° \
                 (위치오차 {:.4} m)",
                best_position_error
            );
        }
    }
}

/// 위치우선 재시도가 실제로 fly_05 표적을 푸는지 직접 확인한다.
#[test]
fn best_normal_solves_the_fly05_target() {
    let arm = pingpong_bot::defaults::primitive_4dof().expect("arm").arm;
    let rail = arm.rail.expect("rail");
    let hint = Pose::new(rail.default_x(), arm.default_joints.clone());
    let desired = Vector3::new(-0.37, 0.89, 0.26).normalize();

    println!("\n=== best_normal 직접 호출 ===");
    for (label, x, y, z) in [
        ("fly_05", 1.585, 0.328, 0.993),
        ("fly_07", 1.652, 0.328, 0.955),
        ("fly_04", 0.640, 0.230, 1.010),
    ] {
        let target = nalgebra::Point3::new(x, y, z);
        match arm.inverse_pose_with_rail_best_normal(target, desired, &hint, IkSearch::Global) {
            Ok((pose, normal_error)) => {
                let actual = arm
                    .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
                    .expect("FK");
                let position_error = (actual.position.coords - target.coords).norm();
                let deg = actual
                    .normal
                    .dot(&desired)
                    .clamp(-1.0, 1.0)
                    .acos()
                    .to_degrees();
                println!(
                    "  {label} → 성공 · 위치오차 {position_error:.5} m · 법선 어긋남 \
                     {deg:.1}° (노름 {normal_error:.3})"
                );
            }
            Err(error) => println!("  {label} → 실패: {error}"),
        }
    }
}
