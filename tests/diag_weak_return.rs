//! 임시 진단: 리턴이 약해 네트를 못 넘는 원인의 컴포넌트 경계 계측.
//!
//! 각 eval 샷마다 다음을 찍는다.
//!   1. 플래너가 원한 v_out  (rally_return_velocity)
//!   2. 임팩트 순간 실제 라켓 접촉점 속도 v_r  (Rapier EE linvel + ω×r)
//!   3. 임팩트 순간 실제 라켓 법선 n
//!   4. 접촉 직전/직후 공 속도 (v_in / v_out 실측)
//!   5. 해석 임팩트 모델이 그 v_r·n 으로 예측하는 v_out_n
//!   6. 실측 v_out 으로 네트 평면에서의 높이

use nalgebra::Vector3;

use pingpong_bot::ball::State;
use pingpong_bot::constants::{BALL_RADIUS, table};
use pingpong_bot::defaults;
use pingpong_bot::eval::{Mode, Protocol};
use pingpong_bot::planner::Impact;
use pingpong_bot::shooter::Settings;
use pingpong_bot::sim::SimWorld;

fn v3(v: rapier3d::prelude::Vector) -> Vector3<f64> {
    return Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z));
}

fn ball_touches(world: &SimWorld, parent: rapier3d::prelude::RigidBodyHandle) -> bool {
    let Some(ball) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(world.ball_handle)).then_some(h))
    else {
        return false;
    };
    let Some(other) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(parent)).then_some(h))
    else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball, other)
        .is_some_and(|p| p.has_any_active_contact());
}

/// 라켓 강체의 **공 접촉점**에서의 속도 = v_com + ω × (p - com).
fn racket_point_velocity(world: &SimWorld, point: Vector3<f64>) -> Vector3<f64> {
    let body = &world.rigid_body_set[world.racket_handle];
    let com = v3(body.center_of_mass());
    let v = v3(body.linvel());
    let w = v3(body.angvel());
    return v + w.cross(&(point - com));
}

/// 라켓 collider 중심(EE 프레임)에서의 실제 속도.
fn racket_ee_velocity(world: &SimWorld) -> Vector3<f64> {
    let (pos, _) = world.racket_pose();
    return racket_point_velocity(world, v3(pos));
}

fn racket_normal(world: &SimWorld) -> Vector3<f64> {
    let (_, rot) = world.racket_pose();
    let z = rot * rapier3d::prelude::Vector::new(0.0, 0.0, 1.0);
    return v3(z).normalize();
}

/// 명령 관절각·관절속도로부터 FK 유한차분으로 라켓 속도를 구한다.
fn commanded_racket_velocity(world: &SimWorld, q: &[f64], qd: &[f64]) -> Option<Vector3<f64>> {
    const H: f64 = 1e-4;
    let arm = world.arm();
    let rail_x = world.robot().rail_x();
    let j0 = pingpong_bot::Joints { values: q.to_vec() };
    let j1 = pingpong_bot::Joints {
        values: q.iter().zip(qd).map(|(a, b)| a + b * H).collect(),
    };
    let p0 = arm
        .forward_kinematics_with_rail(rail_x, &j0)?
        .position
        .coords;
    let p1 = arm
        .forward_kinematics_with_rail(rail_x, &j1)?
        .position
        .coords;
    return Some((p1 - p0) / H);
}

struct ShotDiag {
    index: usize,
    contact: bool,
    v_in: Vector3<f64>,
    v_out_actual: Vector3<f64>,
    v_out_desired: Vector3<f64>,
    v_racket: Vector3<f64>,
    normal: Vector3<f64>,
    impact: Vector3<f64>,
    /// 플래너 궤적이 임팩트 시점에 명령한 관절속도로부터의 라켓 속도.
    v_racket_commanded: Option<Vector3<f64>>,
    normal_commanded: Option<Vector3<f64>>,
    normal_desired: Option<Vector3<f64>>,
    z_at_net: Option<f64>,
    swung: bool,
}

fn run_shot(index: usize, settings: &Settings) -> ShotDiag {
    const DT: f64 = 1.0 / 1000.0;
    const MAX_STEPS: usize = 4_000;

    let robot = defaults::robot().expect("robot");
    let physics = defaults::PhysicsParams::default();
    let mut world = SimWorld::with_physics(robot, physics);
    world.set_use_ground_truth(true);
    world.shoot_ball(settings);

    let mut diag = ShotDiag {
        index,
        contact: false,
        v_in: Vector3::zeros(),
        v_out_actual: Vector3::zeros(),
        v_out_desired: Vector3::zeros(),
        v_racket: Vector3::zeros(),
        normal: Vector3::zeros(),
        impact: Vector3::zeros(),
        v_racket_commanded: None,
        normal_commanded: None,
        normal_desired: None,
        z_at_net: None,
        swung: false,
    };

    let mut prev_v = v3(world.ball_velocity());
    let mut prev_p = v3(world.ball_position());
    let mut in_contact = false;
    let mut contact_done = false;
    let net_y = table::LENGTH_Y * 0.5;

    for _ in 0..MAX_STEPS {
        // 접촉 직전 스텝에서 명령 관절속도를 잡아 둔다.
        let commanded = world.robot().active_swing_sample();
        if commanded.is_some() {
            diag.swung = true;
        }

        world.step(DT, None);

        let p = v3(world.ball_position());
        let v = v3(world.ball_velocity());
        let touching = ball_touches(&world, world.racket_handle);

        if touching && !in_contact && !contact_done {
            in_contact = true;
            diag.contact = true;
            diag.v_in = prev_v;
            diag.impact = p;
            diag.normal = racket_normal(&world);
            diag.v_racket = racket_point_velocity(&world, p);
            diag.v_out_desired = Impact::rally_return(pingpong_bot::Point3::from(p), prev_v);
            if let Some((_, q, qd, _)) = commanded {
                diag.v_racket_commanded = commanded_racket_velocity(&world, &q, &qd);
                diag.normal_commanded = world
                    .arm()
                    .forward_kinematics_with_rail(
                        world.robot().rail_x(),
                        &pingpong_bot::Joints { values: q },
                    )
                    .map(|p| p.normal);
            }
            diag.normal_desired = Some((diag.v_out_desired - prev_v).normalize());
        }
        if in_contact && !touching {
            in_contact = false;
            contact_done = true;
            diag.v_out_actual = v;
        }
        // 네트 평면 통과 높이 (리턴 방향)
        if contact_done && diag.z_at_net.is_none() && prev_p.y < net_y && p.y >= net_y {
            let frac = (net_y - prev_p.y) / (p.y - prev_p.y);
            diag.z_at_net = Some(prev_p.z + (p.z - prev_p.z) * frac);
        }

        prev_v = v;
        prev_p = p;

        if contact_done && world.ball_state == State::Parked {
            break;
        }
        if world.ball_state == State::Parked && diag.swung {
            break;
        }
    }
    return diag;
}

/// 한 발의 스윙 구간 시계열: 명령 라켓속도 vs 실제 라켓속도 vs 공 위치.
#[test]
#[ignore = "진단 전용"]
fn diag_swing_timeseries() {
    const DT: f64 = 1.0 / 1000.0;
    let _ = tracing_subscriber::fmt()
        .with_env_filter("swingdiag=info")
        .without_time()
        .try_init();
    let launch = pingpong_bot::eval::LaunchParams::default();
    let schedule = Protocol::shot_schedule(Mode::Block);
    for pick in [0_usize, 20] {
        let (zone, index_in_zone) = schedule[pick];
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let robot = defaults::robot().expect("robot");
        let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
        world.set_use_ground_truth(true);
        world.shoot_ball(&settings);

        println!("\n=== shot {} ({}) ===", pick + 1, zone.label());
        let mut rows: Vec<(usize, f64, f64, f64, f64, f64, bool, bool, f64)> = Vec::new();
        let mut peak_cmd = 0.0_f64;
        let mut peak_act = 0.0_f64;
        let mut swing_start = None;
        let mut swing_end = None;
        let mut contact_step = None;
        for step in 0..2_000 {
            let commanded = world.robot().active_swing_sample();
            let elapsed = commanded.as_ref().map(|(t, ..)| *t);
            let cmd_v = commanded
                .as_ref()
                .and_then(|(_, q, qd, _)| commanded_racket_velocity(&world, q, qd));
            let swinging = commanded.is_some();
            if swinging && swing_start.is_none() {
                swing_start = Some(step);
            }
            if !swinging && swing_start.is_some() && swing_end.is_none() {
                swing_end = Some(step);
            }
            world.step(DT, None);
            let ball = v3(world.ball_position());
            let act_v = racket_ee_velocity(&world);
            let touch = ball_touches(&world, world.racket_handle);
            if touch && contact_step.is_none() {
                contact_step = Some(step);
            }
            let cmd_norm = cmd_v.map_or(0.0, |v| v.norm());
            peak_cmd = peak_cmd.max(cmd_norm);
            if swinging {
                peak_act = peak_act.max(act_v.norm());
            }
            rows.push((
                step,
                elapsed.unwrap_or(-1.0),
                cmd_norm,
                act_v.norm(),
                cmd_v.map_or(0.0, |v| v.z),
                ball.y,
                touch,
                swinging,
                ball.z,
            ));
            if contact_step.is_some() && step > contact_step.unwrap() + 3 {
                break;
            }
        }
        println!(
            "swing_start={swing_start:?} swing_end={swing_end:?} contact={contact_step:?}  \
             peak cmd|vr|={peak_cmd:.3} peak act|vr|={peak_act:.3}"
        );
        let c = contact_step.unwrap_or(rows.len().saturating_sub(1));
        println!(
            "{:>6} {:>8} {:>9} {:>9} {:>9} {:>8} {:>8} {:>6} {:>5}",
            "step", "elapsed", "cmd|vr|", "act|vr|", "cmd_vz", "ball_y", "ball_z", "touch", "swing"
        );
        for r in rows.iter().filter(|r| r.0 + 90 >= c && r.0 <= c + 3) {
            println!(
                "{:>6} {:>8.3} {:>9.3} {:>9.3} {:>9.3} {:>8.3} {:>8.3} {:>6} {:>5}",
                r.0, r.1, r.2, r.3, r.4, r.5, r.8, r.6, r.7
            );
        }
    }
}

/// 모터 게인 건전성: 스윙 중 관절 추종 오차와 스윙 종료 후 진동.
///
/// eval 점수가 아니라 제어 품질로 감쇠값을 판정하기 위한 계측이다.
/// 과감쇠면 추종 오차가 크고, 저감쇠면 종료 후 부호 반전(진동)이 남는다.
#[test]
#[ignore = "진단 전용"]
fn diag_motor_tracking() {
    const DT: f64 = 1.0 / 1000.0;
    let launch = pingpong_bot::eval::LaunchParams::default();
    let settings =
        Protocol::settings_for_zone_shot(&launch, pingpong_bot::sim::eval_protocol::Zone::Left, 9);
    let mut world = SimWorld::with_physics(
        defaults::robot().expect("robot"),
        defaults::PhysicsParams::default(),
    );
    world.set_use_ground_truth(true);
    world.shoot_ball(&settings);

    let n = world.arm().joint_count();
    let mut peak_err = vec![0.0_f64; n];
    let mut peak_err_during_swing = vec![0.0_f64; n];
    // 스윙 종료 후 관절속도 부호가 몇 번 바뀌는가 = 진동 횟수.
    let mut sign_flips = vec![0_usize; n];
    let mut prev_measured: Option<Vec<f64>> = None;
    let mut prev_dir = vec![0_i32; n];
    let mut swing_ended_at: Option<usize> = None;

    for step in 0..2_500 {
        let swinging = world.robot().is_swinging();
        world.step(DT, None);

        let targets = world.robot().targets().values.clone();
        let measured = world
            .arm_bodies
            .read_joint_angles(&world.multibody_joint_set)
            .values;

        for i in 0..n {
            let err = (targets[i] - measured[i]).abs();
            peak_err[i] = peak_err[i].max(err);
            if swinging {
                peak_err_during_swing[i] = peak_err_during_swing[i].max(err);
            }
        }
        if swinging {
            swing_ended_at = None;
        } else if swing_ended_at.is_none() && step > 50 {
            swing_ended_at = Some(step);
        }
        // 스윙이 끝난 뒤 구간에서만 진동을 센다.
        if let (Some(end), Some(prev)) = (swing_ended_at, prev_measured.as_ref())
            && step > end + 5
        {
            for i in 0..n {
                let d = measured[i] - prev[i];
                let dir = if d > 1e-6 {
                    1
                } else if d < -1e-6 {
                    -1
                } else {
                    0
                };
                if dir != 0 && prev_dir[i] != 0 && dir != prev_dir[i] {
                    sign_flips[i] += 1;
                }
                if dir != 0 {
                    prev_dir[i] = dir;
                }
            }
        }
        prev_measured = Some(measured);
        if world.ball_state == State::Parked && step > 100 {
            break;
        }
    }

    println!("joint  peak_err_swing[rad]  peak_err_all[rad]  post_swing_dir_flips");
    for i in 0..n {
        println!(
            "  q{i}   {:>16.5}   {:>15.5}   {:>18}",
            peak_err_during_swing[i], peak_err[i], sign_flips[i]
        );
    }
}

/// 명중 샷 1발 vs 미스 샷 1발의 입사 궤적 비교 — 바운스 횟수와 위치.
#[test]
#[ignore = "진단 전용"]
fn diag_incoming_trajectory() {
    const DT: f64 = 1.0 / 1000.0;
    let launch = pingpong_bot::eval::LaunchParams::default();

    for (label, zone, index_in_zone) in [
        (
            "#15 Center",
            pingpong_bot::sim::eval_protocol::Zone::Center,
            4,
        ),
        (
            "#13 Center",
            pingpong_bot::sim::eval_protocol::Zone::Center,
            2,
        ),
    ] {
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let mut world = SimWorld::with_physics(
            defaults::robot().expect("robot"),
            defaults::PhysicsParams::default(),
        );
        world.set_use_ground_truth(true);
        world.shoot_ball(&settings);

        println!(
            "\n=== {label}  lateral={:+.3} yaw={:+.2} pitch={:+.2} speed={:.2} ===",
            settings.lateral_offset_m, settings.yaw_deg, settings.pitch_deg, settings.speed_mps
        );
        let mut bounces = 0;
        let mut prev = v3(world.ball_position());
        let mut prev_vz = v3(world.ball_velocity()).z;
        for step in 0..4_000 {
            world.step(DT, None);
            let p = v3(world.ball_position());
            let vz = v3(world.ball_velocity()).z;
            if prev_vz < -0.2 && vz > 0.05 {
                bounces += 1;
                println!(
                    "  bounce #{bounces} @ step {step}  pos=[{:.3} {:.3} {:.3}] vz {:+.2}->{:+.2}",
                    p.x, p.y, p.z, prev_vz, vz
                );
            }
            if world.ball_intersects_net() {
                println!(
                    "  NET CONTACT @ step {step} pos=[{:.3} {:.3} {:.3}]",
                    p.x, p.y, p.z
                );
            }
            if prev.y > table::DEFAULT_HIT_PLANE_Y && p.y <= table::DEFAULT_HIT_PLANE_Y {
                println!(
                    "  hit-plane crossing @ step {step}  pos=[{:.3} {:.3} {:.3}]  bounces so far={bounces}",
                    p.x, p.y, p.z
                );
            }
            prev = p;
            prev_vz = vz;
            if world.ball_state == State::Parked {
                println!(
                    "  parked @ step {step} pos=[{:.3} {:.3} {:.3}]",
                    p.x, p.y, p.z
                );
                break;
            }
        }
    }
}

/// 미접촉 샷의 원인 분해: 예측 오차인가 실행 오차인가.
///
/// 샷마다 다음을 잰다.
///   - 플래너가 커밋한 예측 임팩트점
///   - 공이 히트 평면을 실제로 지날 때의 위치
///   - 공-라켓중심 최소거리와 그 시각의 두 위치
#[test]
#[ignore = "진단 전용"]
fn diag_miss_cause() {
    const DT: f64 = 1.0 / 1000.0;
    let launch = pingpong_bot::eval::LaunchParams::default();
    let hit_plane_y = table::DEFAULT_HIT_PLANE_Y;

    println!(
        "{:>3} {:>6} {:>5} {:>24} {:>24} {:>8} {:>24} {:>24}",
        "#",
        "zone",
        "touch",
        "predicted impact",
        "ball @ hit-plane",
        "min_d",
        "ball @ min_d",
        "racket @ min_d"
    );
    for (i, (zone, index_in_zone)) in Protocol::shot_schedule(Mode::Block).into_iter().enumerate() {
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let mut world = SimWorld::with_physics(
            defaults::robot().expect("robot"),
            defaults::PhysicsParams::default(),
        );
        world.set_use_ground_truth(true);
        world.shoot_ball(&settings);

        let mut predicted: Option<Vector3<f64>> = None;
        let mut at_plane: Option<Vector3<f64>> = None;
        let mut min_d = f64::INFINITY;
        let mut min_ball = Vector3::zeros();
        let mut min_racket = Vector3::zeros();
        let mut touched = false;
        let mut prev_y = v3(world.ball_position()).y;

        for _ in 0..4_000 {
            world.step(DT, None);
            let ball = v3(world.ball_position());
            let racket = v3(world.racket_pose().0);

            if predicted.is_none()
                && world.swing_committed()
                && let Some(p) = world.debug_prediction()
            {
                predicted = Some(p.impact_position.coords);
            }
            let d = (ball - racket).norm();
            if d < min_d {
                min_d = d;
                min_ball = ball;
                min_racket = racket;
            }
            if at_plane.is_none() && prev_y > hit_plane_y && ball.y <= hit_plane_y {
                at_plane = Some(ball);
            }
            if ball_touches(&world, world.racket_handle) {
                touched = true;
            }
            prev_y = ball.y;
            if world.ball_state == State::Parked {
                break;
            }
        }
        let fmt = |v: Option<Vector3<f64>>| {
            v.map(|p| format!("[{:6.3}{:7.3}{:7.3}]", p.x, p.y, p.z))
                .unwrap_or_else(|| format!("{:>21}", "-"))
        };
        println!(
            "{:>3} {:>6} {:>5} {} {} {:8.4} [{:6.3}{:7.3}{:7.3}] [{:6.3}{:7.3}{:7.3}]",
            i + 1,
            zone.label(),
            touched,
            fmt(predicted),
            fmt(at_plane),
            min_d,
            min_ball.x,
            min_ball.y,
            min_ball.z,
            min_racket.x,
            min_racket.y,
            min_racket.z,
        );
    }
}

/// 결정론적 존 샷을 eval 자신의 러너로 돌려 플래그를 찍는다.
/// 진단 하네스의 MISS 가 실제인지 하네스 아티팩트인지 가르기 위한 것.
#[test]
#[ignore = "진단 전용"]
fn diag_eval_flags_deterministic() {
    let launch = pingpong_bot::eval::LaunchParams::default();
    let physics = defaults::PhysicsParams::default();
    let robot = defaults::robot().expect("robot");
    let mut contact = 0;
    for (i, (zone, index_in_zone)) in Protocol::shot_schedule(Mode::Block).into_iter().enumerate() {
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let (flags, passthrough) = Protocol::run_shot(&robot, physics, &settings);
        if flags.contact {
            contact += 1;
        }
        println!(
            "{:>3} {:>6} contact={:<5} cleared_net={:<5} returned_in={:<5} passthrough={:<5} \
             score={}  lateral={:+.3} yaw={:+.2} pitch={:+.2} speed={:.2}",
            i + 1,
            zone.label(),
            flags.contact,
            flags.cleared_net,
            flags.returned_in,
            passthrough,
            flags.score(),
            settings.lateral_offset_m,
            settings.yaw_deg,
            settings.pitch_deg,
            settings.speed_mps,
        );
    }
    println!("\neval-runner contact={contact}/30");
}

#[test]
#[ignore = "진단 전용"]
fn diag_weak_return() {
    let e = defaults::ImpactParams::default().racket_effective_restitution;
    let launch = pingpong_bot::eval::LaunchParams::default();
    let net_top = table::SURFACE_Z + table::NET_HEIGHT + BALL_RADIUS;

    println!(
        "{:>3} {:>6} {:>26} {:>26} {:>26} {:>26} {:>8} {:>8} {:>8} {:>8}",
        "#",
        "zone",
        "v_in",
        "v_out_desired",
        "v_out_actual",
        "v_racket(actual)",
        "vr_cmd",
        "vr_n",
        "vout_n*",
        "z@net"
    );
    let mut cleared = 0;
    let mut contacted = 0;
    for (i, (zone, index_in_zone)) in Protocol::shot_schedule(Mode::Block).into_iter().enumerate() {
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let d = run_shot(i + 1, &settings);
        if !d.contact {
            println!(
                "{:>3} {:>6}  MISS (swung={})",
                d.index,
                zone.label(),
                d.swung
            );
            continue;
        }
        contacted += 1;
        let n = d.normal;
        let vr_n = d.v_racket.dot(&n);
        let vin_n = d.v_in.dot(&n);
        // 해석 모델(무한질량 라켓)이 예측하는 출사 법선속도
        let vout_n_model = (1.0 + e) * vr_n - e * vin_n;
        let vout_n_actual = d.v_out_actual.dot(&n);
        let z_net = d.z_at_net.unwrap_or(f64::NAN);
        if z_net > net_top {
            cleared += 1;
        }
        println!(
            "{:>3} {:>6} [{:6.2}{:6.2}{:6.2}] [{:6.2}{:6.2}{:6.2}] [{:6.2}{:6.2}{:6.2}] [{:6.2}{:6.2}{:6.2}] {:>8} {:8.3} {:8.3}/{:5.2} {:8.3}",
            d.index,
            zone.label(),
            d.v_in.x,
            d.v_in.y,
            d.v_in.z,
            d.v_out_desired.x,
            d.v_out_desired.y,
            d.v_out_desired.z,
            d.v_out_actual.x,
            d.v_out_actual.y,
            d.v_out_actual.z,
            d.v_racket.x,
            d.v_racket.y,
            d.v_racket.z,
            d.v_racket_commanded
                .map(|v| format!("{:.3}", v.norm()))
                .unwrap_or_else(|| "-".into()),
            vr_n,
            vout_n_model,
            vout_n_actual,
            z_net,
        );
        let nd = d.normal_desired.unwrap_or_default();
        let nc = d.normal_commanded.unwrap_or_default();
        println!(
            "      impact=[{:.3} {:.3} {:.3}] |v_out_des|={:.2} |v_out_act|={:.2} |v_r|={:.3} e_eff={:.3}",
            d.impact.x,
            d.impact.y,
            d.impact.z,
            d.v_out_desired.norm(),
            d.v_out_actual.norm(),
            d.v_racket.norm(),
            (vout_n_actual - (1.0 + e) * vr_n).abs() / vin_n.abs(),
        );
        println!(
            "      normal  desired=[{:.3} {:.3} {:.3}]  commanded(FK)=[{:.3} {:.3} {:.3}]  actual=[{:.3} {:.3} {:.3}]  pitch_z: des={:.3} cmd={:.3} act={:.3}",
            nd.x, nd.y, nd.z, nc.x, nc.y, nc.z, n.x, n.y, n.z, nd.z, nc.z, n.z,
        );
    }
    println!("\ncontacted={contacted}/30 cleared_net={cleared} net_top={net_top:.4}");
}
