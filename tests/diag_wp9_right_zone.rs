//! WP9 진단: eval Right 존 30샷 중 10/10 커밋 실패(min_seen_error=inf) 원인 격리.
//!
//! `docs/wp2c-contact-tolerance.md` §7-1, `docs/wp1-hit-plane-window-sweep.md`가
//! 독립적으로 재현한 신호 — Left 10/10, Center 10/10 커밋인데 Right만 0/10,
//! 실패 전부 `min_seen_error = inf`(접촉 오차 검사 도달 전 탈락). 이 파일은
//! 어느 단계에서, 왜 탈락하는지를 실측한다: 목표 좌표 vs 레일 범위, IK
//! 자체의 도달성(레일 이동 시간과 무관하게), 실제 시뮬에서 레일이 커밋
//! 시점까지 목표에 얼마나 접근하는지, `try_auto_swing`이 매 틱 기록한
//! 실패 사유(`SwingPlanError`) 히스토그램.

use pingpong_bot::defaults;
use pingpong_bot::estimator::Prediction;
use pingpong_bot::robot::motion::Planner;
use pingpong_bot::robot;
use pingpong_bot::sim::eval;
use pingpong_bot::sim::physics::SimWorld;

const DT: f64 = 1.0 / 1000.0;
const MAX_STEPS: usize = 4_000;

/// 한 샷의 전체 비행 동안 단계별 계측을 모은다.
struct StageTrace {
    zone_label: &'static str,
    index: usize,
    /// 발사 직후 얻을 수 있는 첫 예측(탄도 초반 추정) 임팩트 x.
    first_prediction_impact_x: Option<f64>,
    /// 커밋 전 마지막(가장 정확한) 예측 임팩트 위치.
    last_prediction_impact: Option<[f64; 3]>,
    /// 레일 x 시계열 요약.
    rail_x_at_launch: f64,
    rail_x_max_reached: f64,
    rail_x_at_commit_or_end: f64,
    /// `try_auto_swing`이 매 틱 남긴 실패 사유 — (표시 문자열, 최초 관측 틱, 횟수).
    fail_histogram: Vec<(String, usize, usize)>,
    swing_committed: bool,
    contact: bool,
    /// 레일 홈(x=0) 기준 목표까지 필요한 거리.
    home_to_target_dist: Option<f64>,
    /// rest 포즈(레일 x=0, 기본 관절)에서 바로 이 임팩트를 잡을 수 있는지 —
    /// 레일 이동 시간과 무관한 순수 IK/조작성 실현 가능성.
    feasible_from_rest: Option<pingpong_bot::robot::motion::Feasibility>,
    /// 레일이 이미 목표 x에 가 있다고 가정했을 때(이동 시간 제거) 실현 가능성.
    feasible_from_target_rail: Option<pingpong_bot::robot::motion::Feasibility>,
}

fn run_stage_trace(zone: eval::Zone, index_in_zone: usize) -> StageTrace {
    let launch = eval::LaunchParams::default();
    let settings = eval::Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
    let robot = defaults::robot().expect("robot");
    let arm = robot.arm.clone();
    let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
    world.set_use_ground_truth(true);

    let rail_x_at_launch = world.robot().rail_x();
    world.shoot_ball(&settings);

    let mut first_prediction_impact_x = None;
    let mut last_prediction_impact = None;
    let mut rail_x_max_reached = rail_x_at_launch;
    let mut fail_histogram: Vec<(String, usize, usize)> = Vec::new();
    let mut contact = false;

    for step in 0..MAX_STEPS {
        world.step(DT, None);

        let rail_x = world.robot().rail_x();
        rail_x_max_reached = rail_x_max_reached.max(rail_x);

        if let Some(p) = world.debug_prediction() {
            if first_prediction_impact_x.is_none() {
                first_prediction_impact_x = Some(p.impact_position.coords.x);
            }
            last_prediction_impact = Some([
                p.impact_position.coords.x,
                p.impact_position.coords.y,
                p.impact_position.coords.z,
            ]);
        }

        if let Some(text) = world.debug_snap().last_fail_text.as_ref() {
            match fail_histogram.iter_mut().find(|(t, ..)| t == text) {
                Some((_, _, count)) => *count += 1,
                None => fail_histogram.push((text.clone(), step, 1)),
            }
        }

        if world.robot().is_swinging() {
            contact = contact || racket_touching_ball(&world);
        }

        if world.swing_committed()
            && !world.robot().is_swinging()
            && step > 10
            && world.ball_state == pingpong_bot::sim::physics::BallState::Parked
        {
            break;
        }
        if world.ball_state == pingpong_bot::sim::physics::BallState::Parked && step > 200 {
            break;
        }
    }

    let rail = arm.rail.expect("rail");
    let home_to_target_dist = last_prediction_impact.map(|p| (p[0] - rail.home_x()).abs());

    let feasible_from_rest = last_prediction_impact.and_then(|p| {
        let prediction = synth_prediction(&world, p);
        let start = robot::Pose::new(rail.home_x(), arm.default_joints.clone());
        Planner::feasibility(&arm, &prediction, &start)
    });
    let feasible_from_target_rail = last_prediction_impact.and_then(|p| {
        let prediction = synth_prediction(&world, p);
        let target_rail_x = rail.clamp_x(p[0]);
        let start = robot::Pose::new(target_rail_x, arm.default_joints.clone());
        Planner::feasibility(&arm, &prediction, &start)
    });

    return StageTrace {
        zone_label: zone.label(),
        index: index_in_zone,
        first_prediction_impact_x,
        last_prediction_impact,
        rail_x_at_launch,
        rail_x_max_reached,
        rail_x_at_commit_or_end: world.robot().rail_x(),
        fail_histogram,
        swing_committed: world.swing_committed(),
        contact,
        home_to_target_dist,
        feasible_from_rest,
        feasible_from_target_rail,
    };
}

/// `debug_prediction()`에서 남은 `incoming_velocity`를 재사용해 합성
/// `Prediction`을 만든다 — `Planner::feasibility`는 `time_to_impact_secs`를
/// 안 쓰므로(내부적으로 IK/조작성만 봄) 0으로 둬도 무방.
fn synth_prediction(world: &SimWorld, impact_xyz: [f64; 3]) -> Prediction {
    let v_in = world
        .debug_prediction()
        .map(|p| p.incoming_velocity)
        .unwrap_or_default();
    return Prediction {
        time_to_impact_secs: 0.0,
        impact_position: pingpong_bot::Point3::new(impact_xyz[0], impact_xyz[1], impact_xyz[2]),
        incoming_velocity: v_in,
    };
}

fn racket_touching_ball(world: &SimWorld) -> bool {
    let Some(ball) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(world.ball_handle)).then_some(h))
    else {
        return false;
    };
    let Some(racket) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(world.racket_handle)).then_some(h))
    else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball, racket)
        .is_some_and(|p| p.has_any_active_contact());
}

fn print_trace(t: &StageTrace) {
    println!(
        "\n=== {} #{} === commit={} contact={}",
        t.zone_label,
        t.index + 1,
        t.swing_committed,
        t.contact
    );
    println!(
        "  rail_x: launch={:.4}  max_reached={:.4}  at_end={:.4}",
        t.rail_x_at_launch, t.rail_x_max_reached, t.rail_x_at_commit_or_end
    );
    println!(
        "  prediction impact_x: first={:?}  last={:?}  home_to_target_dist={:?}",
        t.first_prediction_impact_x, t.last_prediction_impact, t.home_to_target_dist
    );
    println!("  feasible_from_rest (rail already at home x=0): {:?}", t.feasible_from_rest);
    println!(
        "  feasible_from_target_rail (rail pre-positioned at target x): {:?}",
        t.feasible_from_target_rail
    );
    if t.fail_histogram.is_empty() {
        println!("  fail_histogram: (없음 — 실패 기록 없이 커밋됨)");
    } else {
        println!("  fail_histogram (사유, 최초관측 틱, 횟수):");
        for (text, first_step, count) in &t.fail_histogram {
            println!("    [{count:>4}x from step {first_step:>4}] {text}");
        }
    }
}

/// 메인 진단: eval 30샷 스케줄에서 각 존 앞 3발씩(지터 없음, 결정론)을 돌려
/// 실패 단계·사유·좌표를 찍는다.
#[test]
#[ignore = "진단 전용 (WP9)"]
fn diag_wp9_right_zone_stage_trace() {
    for zone in [eval::Zone::Left, eval::Zone::Center, eval::Zone::Right] {
        for index_in_zone in 0..3 {
            let trace = run_stage_trace(zone, index_in_zone);
            print_trace(&trace);
        }
    }
}

/// 확인 실험: 레일 초기 위치를 `home_x()`(x=0, 극단 가장자리) 대신
/// `default_x()`(테이블 중앙, `plan_return_to_center`가 매 스윙 뒤 실제로
/// 돌아가는 곳)로 바꾸면 Right 존이 커밋되는가?
///
/// 이게 성공하면: Right 탈락은 팔/레일의 물리적 도달성 문제가 아니라,
/// "매 eval 샷마다 새 SimWorld를 만들어 `Arm::initial_state()`(레일
/// `home_x()`=0)에서 시작한다"는 부트 상태 아티팩트가 원인이라는 직접 증거.
#[test]
#[ignore = "진단 전용 (WP9)"]
fn diag_wp9_right_zone_commits_from_centered_start() {
    let launch = eval::LaunchParams::default();
    for zone in [eval::Zone::Left, eval::Zone::Center, eval::Zone::Right] {
        for index_in_zone in 0..3 {
            let settings = eval::Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
            let robot = defaults::robot().expect("robot");
            let center_rail_x = robot.arm.rail.expect("rail").default_x();
            let default_joints = robot.arm.default_joints.clone();
            let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
            world.set_use_ground_truth(true);
            // 레일을 home(x=0) 대신 중앙(테이블 폭 절반)에서 출발시킨다 —
            // `plan_return_to_center`가 실제로 스윙 뒤 돌아가는 위치와 동일.
            *world.robot_mut() = robot::State::new(default_joints, center_rail_x);
            world.shoot_ball(&settings);

            let mut committed = false;
            let mut contact = false;
            for _ in 0..MAX_STEPS {
                world.step(DT, None);
                committed = committed || world.swing_committed();
                if world.robot().is_swinging() {
                    contact = contact || racket_touching_ball(&world);
                }
                if world.ball_state == pingpong_bot::sim::physics::BallState::Parked {
                    break;
                }
            }
            println!(
                "{:>6} #{}: rail_start=center({:.4})  commit={}  contact={}",
                zone.label(),
                index_in_zone + 1,
                center_rail_x,
                committed,
                contact
            );
        }
    }
}

/// 레일 홈 위치(x=0)와 테이블 폭/각 존 목표 x의 기하학적 관계 — 좌우 비대칭이
/// "레일 도달 범위"가 아니라 "홈 위치에서 목표까지의 이동 거리" 차이에서
/// 오는지 수치로 확인한다.
#[test]
#[ignore = "진단 전용 (WP9)"]
fn diag_wp9_home_position_asymmetry() {
    let robot = defaults::robot().expect("robot");
    let rail = robot.arm.rail.expect("rail");
    println!(
        "rail: x_min(home)={:.4} x_max={:.4} default_x(center)={:.4} max_speed={:.3}",
        rail.x_min, rail.x_max, rail.x_max, rail.max_speed
    );
    println!(
        "rail default_x (table center) = {:.4}, home_x = {:.4}, distance home->center = {:.4}",
        rail.default_x(),
        rail.home_x(),
        (rail.default_x() - rail.home_x()).abs()
    );

    for zone in [eval::Zone::Left, eval::Zone::Center, eval::Zone::Right] {
        let launch = eval::LaunchParams::default();
        let settings = eval::Protocol::settings_for_zone_shot(&launch, zone, 0);
        let mut world = SimWorld::with_physics(
            defaults::robot().expect("robot"),
            defaults::PhysicsParams::default(),
        );
        world.set_use_ground_truth(true);
        world.shoot_ball(&settings);
        // 몇 스텝 돌려 예측이 안정화되도록.
        let mut impact_x = None;
        for _ in 0..50 {
            world.step(DT, None);
            if let Some(p) = world.debug_prediction() {
                impact_x = Some(p.impact_position.coords.x);
            }
        }
        println!(
            "{:>6}: yaw={:+.2} deg  early impact_x≈{:?}  dist_from_home={:?}",
            zone.label(),
            settings.yaw_deg,
            impact_x,
            impact_x.map(|x: f64| (x - rail.home_x()).abs())
        );
    }
}
