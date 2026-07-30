//! WP4b 진단: WP5(coarse rate-limit) 적용 이후에도 실제 시뮬레이션 중
//! 테이블 관통이 재현되는지 확인한다.
//!
//! 기존 `planner::collision::tests`(고정 자세)는 여전히 통과하지만, 사용자가
//! 관찰한 관통은 GUI에서 실시간 스윙 중이었다 — **명령된 목표**가 아니라
//! **Rapier가 실제로 추종한 관절각**을 전 스윙 구간에서 매 물리 틱 샘플링해
//! `table_penetration`을 잰다. WP5가 coarse 목표에는 `clamp_above_table`을
//! 적용했지만, PD 추종 중 오버슈트/토크 포화로 실제 자세가 순간적으로
//! 파고들 가능성은 남아 있다.

use pingpong_bot::defaults;
use pingpong_bot::robot::collision::table_penetration;
use pingpong_bot::sim::eval::{LaunchParams as EvalLaunchParams, Mode as EvalMode, Protocol};
use pingpong_bot::sim::launch::Settings as BallShooterSettings;
use pingpong_bot::sim::physics::{BallState, SimWorld};

const DT: f64 = 1.0 / 1000.0;
const MAX_STEPS: usize = 4_000;

struct PenetrationResult {
    label: String,
    worst_depth: f64,
    worst_step: usize,
    worst_phase: &'static str,
}

fn sweep_shot(label: String, settings: &BallShooterSettings) -> PenetrationResult {
    let mut world = SimWorld::with_physics(
        defaults::robot().expect("robot"),
        defaults::PhysicsParams::default(),
    );
    world.set_use_ground_truth(true);
    world.shoot_ball(settings);

    let mut worst_depth = 0.0_f64;
    let mut worst_step = 0;
    let mut worst_phase: &'static str = "none";

    for step in 0..MAX_STEPS {
        let swinging_before = world.robot().is_swinging();
        world.step(DT, None);
        let measured = world
            .arm_bodies
            .read_joint_angles(&world.multibody_joint_set)
            .values;
        let joints = pingpong_bot::robot::Joints { values: measured };
        let rail_x = world.robot().rail_x();
        let depth = table_penetration(world.arm(), rail_x, &joints);
        if depth > worst_depth {
            worst_depth = depth;
            worst_step = step;
            worst_phase = if swinging_before {
                "swing"
            } else if world.robot().is_swinging() {
                "swing_start"
            } else {
                "coarse_or_idle"
            };
        }
        if world.ball_state == BallState::Parked && step > 100 {
            break;
        }
    }
    return PenetrationResult {
        label,
        worst_depth,
        worst_step,
        worst_phase,
    };
}

/// eval 30샷 격자 전수 — 실제 추종 자세 기준 최대 관통 깊이.
#[test]
#[ignore = "진단 전용"]
fn diag_table_penetration_live_eval_grid() {
    let launch = EvalLaunchParams::default();
    let results: Vec<PenetrationResult> = Protocol::shot_schedule(EvalMode::Block)
        .into_iter()
        .enumerate()
        .map(|(i, (zone, index_in_zone))| {
            let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
            sweep_shot(format!("{}#{}", zone.label(), i + 1), &settings)
        })
        .collect();

    println!(
        "{:>10} {:>10} {:>8} {:>14}",
        "shot", "worst_m", "step", "phase"
    );
    let mut any_penetration = false;
    for r in &results {
        if r.worst_depth > 1e-4 {
            any_penetration = true;
        }
        println!(
            "{:>10} {:>10.5} {:>8} {:>14}",
            r.label, r.worst_depth, r.worst_step, r.worst_phase
        );
    }
    let worst = results
        .iter()
        .map(|r| r.worst_depth)
        .fold(0.0_f64, f64::max);
    println!("\nworst overall depth = {worst:.5} m  any_penetration={any_penetration}");
}

/// 랜덤 샷 격자 — 좌우/yaw 스윕으로 더 다양한 임팩트 자세를 본다.
#[test]
#[ignore = "진단 전용"]
fn diag_table_penetration_live_random_grid() {
    let mut results = Vec::new();
    for lateral in [-0.5, -0.25, 0.0, 0.25, 0.5] {
        for yaw in [-15.0, -7.5, 0.0, 7.5, 15.0] {
            let settings = BallShooterSettings {
                lateral_offset_m: lateral,
                yaw_deg: yaw,
                ..BallShooterSettings::default()
            };
            results.push(sweep_shot(
                format!("lat{lateral:+.2}_yaw{yaw:+.1}"),
                &settings,
            ));
        }
    }
    println!(
        "{:>18} {:>10} {:>8} {:>14}",
        "shot", "worst_m", "step", "phase"
    );
    let mut any_penetration = false;
    for r in &results {
        if r.worst_depth > 1e-4 {
            any_penetration = true;
        }
        println!(
            "{:>18} {:>10.5} {:>8} {:>14}",
            r.label, r.worst_depth, r.worst_step, r.worst_phase
        );
    }
    let worst = results
        .iter()
        .map(|r| r.worst_depth)
        .fold(0.0_f64, f64::max);
    println!("\nworst overall depth = {worst:.5} m  any_penetration={any_penetration}");
}

/// 양성 대조군 — 계측 함수가 실제로 관통을 감지할 수 있는지 확인.
///
/// `defaults::robot()`의 마운트가 `planner::collision::tests`가 쓰는
/// `primitive_4dof()`와 달라 그 테스트의 고정 자세가 이 로봇에서는 애초에
/// 관통이 아니다 — 그 테스트 자체의 폴백(테이블 면 아래 지점으로 IK 강제)을
/// 그대로 재사용해 이 로봇 기준으로 확실히 관통하는 자세를 만든다.
#[test]
fn diag_table_penetration_live_positive_control() {
    let robot = defaults::robot().expect("robot");
    let arm = &robot.arm;
    let rail_x = arm.rail.as_ref().map(|r| r.home_x()).unwrap_or(0.0);
    let below = pingpong_bot::Point3::new(
        rail_x,
        pingpong_bot::constants::table::DEFAULT_HIT_PLANE_Y,
        pingpong_bot::constants::table::SURFACE_Z - 0.05,
    );
    let bad_joints = if let Some(rail) = &arm.rail {
        arm.inverse_kinematics_with_rail(rail, rail_x, below, Some(&arm.default_joints))
    } else {
        arm.inverse_kinematics_near(below, Some(&arm.default_joints))
    }
    .expect("테이블 아래 지점 IK");
    let depth = table_penetration(arm, rail_x, &bad_joints);
    println!("positive control depth = {depth}");
    assert!(depth > 0.0, "계측 함수가 알려진 관통 자세를 못 잡음: depth={depth}");
}
