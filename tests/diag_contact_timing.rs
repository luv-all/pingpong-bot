//! WP6 진단: 실제 Rapier `ContactPair` 발동 시각 vs 플래너 `impact_time_secs`.
//!
//! `swing_bench --sim-verify`가 보고하는 "실제 접촉이 계획보다 ~4.7ms 이르다"를
//! 성분별로 분해한다. 같은 시계(`elapsed` = 커밋 틱 종료 후 경과 sim 시간,
//! `swing_bench`의 `run_sim_verify`와 동일한 관례)에서 세 시각을 잰다.
//!
//!   1. `planned`      — `trajectory.impact_time_secs` (플래너가 목표한 임팩트)
//!   2. `plane_cross`  — 공이 예측 임팩트 y평면을 실제로 지나는 시각 (선형보간)
//!   3. `contact`      — Rapier `ContactPair::has_any_active_contact`가 처음 참
//!
//! 분해:
//!   - `plane_cross - planned` → **예측기 오차**(적분 정합, 가설 b). 프레임
//!     관례상 정확한 예측기라도 `-dt`가 나오는 게 정답이다(아래 주석 참고).
//!   - `contact - plane_cross` → **기하 스윕 오차**(가설 a/c). 라켓 면이
//!     15×16 cm 박스로 움직이며 접근하므로 점-평면 교차보다 이를 수 있다.
//!
//! 프레임 관례: `try_auto_swing`은 `step()` 안에서 Rapier 적분 **전에** 돌아
//! 그 틱 시작 시점의 공 상태로 `time_to_impact_secs`를 계산하고, 궤적 재생을
//! `elapsed=0`으로 시작한다. 그런데 그 틱은 이미 공을 한 번 적분한 뒤 끝난다.
//! 따라서 "커밋 틱 종료" 기준 시계에서 공이 평면에 닿는 시각은 `tti - dt`다.
//! 즉 `plane_cross - planned ≈ -dt`(= -1 ms)가 **오차 0**에 해당한다.

use nalgebra::Vector3;

use pingpong_bot::constants::geometry::{RACKET_HALF_X, RACKET_HALF_Y, RACKET_HALF_Z};
use pingpong_bot::constants::{BALL_RADIUS, table};
use pingpong_bot::defaults;
use pingpong_bot::estimator::Impact;
use pingpong_bot::sim::eval::{LaunchParams as EvalLaunchParams, Mode as EvalMode, Protocol, Zone as EvalZone};
use pingpong_bot::sim::launch::Settings as BallShooterSettings;
use pingpong_bot::sim::physics::{BallState, SimWorld};

const DT: f64 = 1.0 / 1000.0;
const MAX_WAIT_STEPS: usize = 4_000;
const MAX_CONTACT_STEPS: usize = 3_000;

fn v3(v: rapier3d::prelude::Vector) -> Vector3<f64> {
    return Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z));
}

fn ball_touches_racket(world: &SimWorld) -> bool {
    let collider_for = |parent| {
        world
            .collider_set
            .iter()
            .find_map(|(h, c)| (c.parent() == Some(parent)).then_some(h))
    };
    let Some(ball) = collider_for(world.ball_handle) else {
        return false;
    };
    let Some(racket) = collider_for(world.racket_handle) else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball, racket)
        .is_some_and(|p| p.has_any_active_contact());
}

/// 라켓 collider 프레임 (중심, local x/y/normal 축).
fn racket_frame(world: &SimWorld) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>, Vector3<f64>) {
    let (pos, rot) = world.racket_pose();
    let axis = |x, y, z| v3(rot * rapier3d::prelude::Vector::new(x, y, z)).normalize();
    return (
        v3(pos),
        axis(1.0, 0.0, 0.0),
        axis(0.0, 1.0, 0.0),
        axis(0.0, 0.0, 1.0),
    );
}

/// 라켓 강체의 `point`에서의 속도 = v_com + ω × (p − com).
fn racket_point_velocity(world: &SimWorld, point: Vector3<f64>) -> Vector3<f64> {
    let body = &world.rigid_body_set[world.racket_handle];
    let com = v3(body.center_of_mass());
    return v3(body.linvel()) + v3(body.angvel()).cross(&(point - com));
}

/// 공 표면과 라켓 박스 표면 사이 간격 [m] (음수 = 관통). Rapier `ContactPair`가
/// 무엇을 보고 발동하는지와 같은 기하를 손으로 계산해 **틱 사이 보간**을
/// 가능하게 한다 (틱 해상도 1 ms로는 4.7 ms를 성분 분해할 수 없다).
struct FaceGap {
    gap: f64,
    /// 면 내 접촉 좌표 [m] (0 = 면 중심). |u|→RACKET_HALF_X이면 가장자리 접촉.
    u: f64,
    v: f64,
    /// 법선 방향 거리 [m].
    w: f64,
}

fn face_gap(world: &SimWorld, ball: Vector3<f64>) -> FaceGap {
    let (center, ex, ey, n) = racket_frame(world);
    let d = ball - center;
    let (u, v, w) = (d.dot(&ex), d.dot(&ey), d.dot(&n));
    let closest = Vector3::new(
        u.clamp(-RACKET_HALF_X, RACKET_HALF_X),
        v.clamp(-RACKET_HALF_Y, RACKET_HALF_Y),
        w.clamp(-RACKET_HALF_Z, RACKET_HALF_Z),
    );
    let local = Vector3::new(u, v, w);
    return FaceGap {
        gap: (local - closest).norm() - BALL_RADIUS,
        u,
        v,
        w,
    };
}

#[derive(Debug)]
struct TimingRow {
    label: String,
    launch_speed: f64,
    committed: bool,
    planned_secs: f64,
    /// 공이 예측 임팩트 y평면을 지나는 시각 (선형보간).
    plane_cross_secs: Option<f64>,
    /// Rapier `ContactPair` 최초 활성 틱.
    contact_secs: Option<f64>,
    /// 손계산 face-gap 부호 반전을 선형보간한 접촉 시각 — 틱 이하 해상도.
    contact_interp_secs: Option<f64>,
    /// 접촉 순간 라켓 접촉점 속도 크기 [m/s].
    racket_speed: f64,
    /// 접촉 순간 법선 방향 접근속도 [m/s] (양수 = 서로 다가옴).
    closing_speed_n: f64,
    /// 접촉 순간 라켓 속도의 법선 성분 [m/s] (라켓이 면 법선으로 얼마나 빠르게).
    racket_speed_n: f64,
    /// 접촉 순간 면 내 좌표 [m] — 가장자리 접촉인지 중앙 접촉인지.
    contact_u: f64,
    contact_v: f64,
    /// 접촉 순간 법선 거리 [m] (계획값 = BALL_RADIUS + RACKET_HALF_Z).
    contact_w: f64,
}

fn run_shot(label: String, launch_speed: f64, settings: &BallShooterSettings) -> TimingRow {
    return run_shot_tweaked(label, launch_speed, settings, |_| {});
}

/// [`run_shot`] + Rapier `integration_parameters` 조정 훅 — 팀리드 후속 실험:
/// `diag_table_restitution`의 `normalized_prediction_distance` 단서가 공-라켓
/// 접촉 타이밍(`d_total`)에도 같은 효과를 내는지 `shoot_ball` 전에 솔버 노브를
/// 바꿔 확인한다.
fn run_shot_tweaked(
    label: String,
    launch_speed: f64,
    settings: &BallShooterSettings,
    tweak: impl Fn(&mut SimWorld),
) -> TimingRow {
    let mut world = SimWorld::with_physics(
        defaults::robot().expect("robot"),
        defaults::PhysicsParams::default(),
    );
    tweak(&mut world);
    world.set_use_ground_truth(true);
    world.shoot_ball(settings);

    let mut row = TimingRow {
        label,
        launch_speed,
        committed: false,
        planned_secs: f64::NAN,
        plane_cross_secs: None,
        contact_secs: None,
        contact_interp_secs: None,
        racket_speed: f64::NAN,
        closing_speed_n: f64::NAN,
        racket_speed_n: f64::NAN,
        contact_u: f64::NAN,
        contact_v: f64::NAN,
        contact_w: f64::NAN,
    };

    // 1) 커밋까지 — `swing_bench::run_sim_verify`와 같은 루프.
    let mut committed = None;
    for _ in 0..MAX_WAIT_STEPS {
        world.step(DT, None);
        if world.robot().is_swinging()
            && let Some(trajectory) = world.robot().active_trajectory()
        {
            let plane_y = world
                .debug_prediction()
                .map(|p| p.impact_position.coords.y)
                .unwrap_or(table::DEFAULT_HIT_PLANE_Y);
            committed = Some((trajectory.impact_time_secs, plane_y));
            break;
        }
    }
    let Some((planned_secs, plane_y)) = committed else {
        return row;
    };
    row.committed = true;
    row.planned_secs = planned_secs;

    // 2) 커밋 이후 매 틱 계측.
    let mut elapsed = 0.0;
    let mut prev_ball = v3(world.ball_position());
    let mut prev_gap: Option<(f64, f64)> = None; // (elapsed, gap)
    for _ in 0..MAX_CONTACT_STEPS {
        world.step(DT, None);
        elapsed += DT;
        let ball = v3(world.ball_position());
        let g = face_gap(&world, ball);

        if row.plane_cross_secs.is_none() && prev_ball.y > plane_y && ball.y <= plane_y {
            let denom = ball.y - prev_ball.y;
            let frac = if denom.abs() < 1e-12 {
                0.0
            } else {
                (plane_y - prev_ball.y) / denom
            };
            row.plane_cross_secs = Some(elapsed - DT + DT * frac);
        }

        if row.contact_interp_secs.is_none()
            && g.gap <= 0.0
            && let Some((prev_t, prev_g)) = prev_gap
            && prev_g > 0.0
        {
            let frac = prev_g / (prev_g - g.gap);
            row.contact_interp_secs = Some(prev_t + (elapsed - prev_t) * frac);
        }

        if row.contact_secs.is_none() && ball_touches_racket(&world) {
            row.contact_secs = Some(elapsed);
            let (_, _, _, n) = racket_frame(&world);
            let v_racket = racket_point_velocity(&world, ball);
            let v_ball = v3(world.ball_velocity());
            row.racket_speed = v_racket.norm();
            row.racket_speed_n = v_racket.dot(&n);
            row.closing_speed_n = (v_racket - v_ball).dot(&n);
            row.contact_u = g.u;
            row.contact_v = g.v;
            row.contact_w = g.w;
        }

        prev_gap = Some((elapsed, g.gap));
        prev_ball = ball;
        if row.contact_secs.is_some() && elapsed > row.contact_secs.unwrap() + 0.01 {
            break;
        }
        if world.ball_state == BallState::Parked {
            break;
        }
    }
    return row;
}

fn print_table(rows: &[TimingRow]) {
    println!(
        "{:>14} {:>6} {:>8} {:>9} {:>9} {:>9} {:>9} {:>9} {:>7} {:>7} {:>7} {:>7} {:>7}",
        "shot",
        "v_lnch",
        "planned",
        "plane_x",
        "contact",
        "d_pred",
        "d_geom",
        "d_total",
        "|v_r|",
        "v_r·n",
        "close",
        "u",
        "w"
    );
    for r in rows {
        if !r.committed {
            println!("{:>14} {:>6.1}  no-commit", r.label, r.launch_speed);
            continue;
        }
        let Some(contact) = r.contact_secs else {
            println!(
                "{:>14} {:>6.1} {:>8.4}  committed but NO CONTACT",
                r.label, r.launch_speed, r.planned_secs
            );
            continue;
        };
        // 프레임 관례 보정: 정확한 예측기는 `plane_cross = planned - dt`.
        let d_pred = r.plane_cross_secs.map(|t| t - (r.planned_secs - DT));
        let d_geom = r
            .plane_cross_secs
            .and_then(|p| r.contact_interp_secs.map(|c| c - p));
        let d_total = contact - r.planned_secs;
        let f = |v: Option<f64>| v.map_or("      -".to_string(), |x| format!("{:+7.4}", x * 1.0));
        println!(
            "{:>14} {:>6.1} {:>8.4} {:>9} {:>9.4} {:>9} {:>9} {:>+9.4} {:>7.3} {:>7.3} {:>7.3} {:>+7.4} {:>7.4}",
            r.label,
            r.launch_speed,
            r.planned_secs,
            r.plane_cross_secs
                .map_or("      -".to_string(), |t| format!("{t:7.4}")),
            contact,
            f(d_pred),
            f(d_geom),
            d_total,
            r.racket_speed,
            r.racket_speed_n,
            r.closing_speed_n,
            r.contact_u,
            r.contact_w,
        );
    }
    let planned_target = BALL_RADIUS + RACKET_HALF_Z;
    println!("(계획 접촉 법선거리 w = BALL_RADIUS + RACKET_HALF_Z = {planned_target:.4} m)");

    let usable: Vec<&TimingRow> = rows
        .iter()
        .filter(|r| r.committed && r.contact_secs.is_some())
        .collect();
    if usable.is_empty() {
        return;
    }
    let mean = |f: &dyn Fn(&TimingRow) -> f64| -> f64 {
        return usable.iter().map(|r| f(r)).sum::<f64>() / usable.len() as f64;
    };
    let d_total = |r: &TimingRow| r.contact_secs.unwrap() - r.planned_secs;
    let d_geom = |r: &TimingRow| match (r.plane_cross_secs, r.contact_interp_secs) {
        (Some(p), Some(c)) => c - p,
        _ => f64::NAN,
    };
    let d_pred = |r: &TimingRow| match r.plane_cross_secs {
        Some(p) => p - (r.planned_secs - DT),
        None => f64::NAN,
    };
    println!(
        "\nn={} mean d_total={:+.5}s  mean d_pred={:+.5}s  mean d_geom={:+.5}s",
        usable.len(),
        mean(&d_total),
        mean(&d_pred),
        mean(&d_geom),
    );

    // 속도 의존성: gap이 접근속도에 비례(기하)하는지 무관(적분)한지.
    let corr = |x: &dyn Fn(&TimingRow) -> f64, y: &dyn Fn(&TimingRow) -> f64| -> f64 {
        let pts: Vec<(f64, f64)> = usable
            .iter()
            .map(|r| (x(r), y(r)))
            .filter(|(a, b)| a.is_finite() && b.is_finite())
            .collect();
        if pts.len() < 3 {
            return f64::NAN;
        }
        let n = pts.len() as f64;
        let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
        let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
        let cov: f64 = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
        let sx: f64 = pts.iter().map(|p| (p.0 - mx).powi(2)).sum::<f64>().sqrt();
        let sy: f64 = pts.iter().map(|p| (p.1 - my).powi(2)).sum::<f64>().sqrt();
        if sx < 1e-12 || sy < 1e-12 {
            return f64::NAN;
        }
        return cov / (sx * sy);
    };
    println!(
        "corr(closing_speed, d_total)={:+.3}  corr(closing_speed, d_geom)={:+.3}  \
         corr(|v_r|, d_geom)={:+.3}",
        corr(&|r| r.closing_speed_n, &d_total),
        corr(&|r| r.closing_speed_n, &d_geom),
        corr(&|r| r.racket_speed, &d_geom),
    );
}

/// 대표 샷 한 발의 접촉 직전 틱별 궤적 — "공이 예측 임팩트점에 도달하기
/// 전에 왜 접촉이 발동하는가"를 공간적으로 본다.
///
/// 매 틱: 공 위치, 예측 임팩트점 대비 편차, Rapier 라켓 중심, 궤적이 그 시각에
/// **명령한** 라켓 중심(FK), 면 좌표 (u, v, w), face gap.
#[test]
#[ignore = "진단 전용"]
fn diag_contact_timing_trace() {
    let launch = EvalLaunchParams::default();
    for (zone, index_in_zone) in [(EvalZone::Center, 0), (EvalZone::Left, 2)] {
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let mut world = SimWorld::with_physics(
            defaults::robot().expect("robot"),
            defaults::PhysicsParams::default(),
        );
        world.set_use_ground_truth(true);
        world.shoot_ball(&settings);

        let mut committed = None;
        for _ in 0..MAX_WAIT_STEPS {
            world.step(DT, None);
            if world.robot().is_swinging()
                && let Some(trajectory) = world.robot().active_trajectory()
            {
                committed = Some((
                    trajectory.clone(),
                    world.debug_prediction().copied().expect("커밋 예측"),
                ));
                break;
            }
        }
        let Some((trajectory, prediction)) = committed else {
            println!("\n=== {}/{index_in_zone}: no-commit ===", zone.label());
            continue;
        };
        let impact = prediction.impact_position.coords;
        let normal = {
            let v_out = Impact::rally_return(
                prediction.impact_position,
                prediction.incoming_velocity,
            );
            (v_out - prediction.incoming_velocity).normalize()
        };
        let planned_center = impact - normal * (BALL_RADIUS + RACKET_HALF_Z);
        println!(
            "\n=== {}/{index_in_zone}  planned impact_time={:.4}s  impact=[{:.4} {:.4} {:.4}] \
             n=[{:.3} {:.3} {:.3}]\n    planned racket center=[{:.4} {:.4} {:.4}] tti={:.4} ===",
            zone.label(),
            trajectory.impact_time_secs,
            impact.x,
            impact.y,
            impact.z,
            normal.x,
            normal.y,
            normal.z,
            planned_center.x,
            planned_center.y,
            planned_center.z,
            prediction.time_to_impact_secs,
        );
        println!(
            "{:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>6}",
            "elapsed", "ball_y", "d_ball_y", "d_ball_z", "cen_y", "d_cen_y", "cmd_y", "u", "v", "gap", "touch"
        );
        let mut elapsed = 0.0;
        let mut rows: Vec<String> = Vec::new();
        let mut contact_at: Option<usize> = None;
        for _ in 0..MAX_CONTACT_STEPS {
            world.step(DT, None);
            elapsed += DT;
            let ball = v3(world.ball_position());
            let g = face_gap(&world, ball);
            let (center, ..) = racket_frame(&world);
            let commanded_center = world
                .arm()
                .forward_kinematics_with_rail(
                    trajectory.sample_rail_at(elapsed),
                    &trajectory.sample_at(elapsed),
                )
                .map(|p| p.position.coords);
            let touch = ball_touches_racket(&world);
            if touch && contact_at.is_none() {
                contact_at = Some(rows.len());
            }
            rows.push(format!(
                "{elapsed:>8.4} {:>8.4} {:>+8.4} {:>+8.4} {:>8.4} {:>+8.4} {:>8} {:>+8.4} {:>+8.4} {:>+8.5} {:>6}",
                ball.y,
                ball.y - impact.y,
                ball.z - impact.z,
                center.y,
                center.y - planned_center.y,
                commanded_center.map_or("-".to_string(), |c| format!("{:8.4}", c.y)),
                g.u,
                g.v,
                g.gap,
                touch,
            ));
            if let Some(c) = contact_at
                && rows.len() > c + 2
            {
                break;
            }
            if world.ball_state == BallState::Parked {
                break;
            }
        }
        let c = contact_at.unwrap_or(rows.len().saturating_sub(1));
        for row in rows.iter().skip(c.saturating_sub(20)) {
            println!("{row}");
        }
    }
}

/// 커밋 시점 공 상태에서 예측기 커널(`semi_implicit_euler`)을 Rapier와
/// **락스텝**으로 굴려 발산을 성분별로 본다 — 5 cm z 오차가 어디서 생기는지.
#[test]
#[ignore = "진단 전용"]
fn diag_predictor_vs_rapier_divergence() {
    let launch = EvalLaunchParams::default();
    let physics = defaults::PhysicsParams::default();
    for (zone, index_in_zone) in [(EvalZone::Center, 0), (EvalZone::Left, 2)] {
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let mut world = SimWorld::with_physics(defaults::robot().expect("robot"), physics);
        world.set_use_ground_truth(true);
        world.shoot_ball(&settings);

        let mut planned = None;
        for _ in 0..MAX_WAIT_STEPS {
            world.step(DT, None);
            if world.robot().is_swinging()
                && let Some(t) = world.robot().active_trajectory()
            {
                planned = Some((
                    t.impact_time_secs,
                    world.debug_prediction().copied().expect("커밋 예측"),
                ));
                break;
            }
        }
        let Some((impact_time, prediction)) = planned else {
            println!("\n=== {}/{index_in_zone}: no-commit ===", zone.label());
            continue;
        };

        // 커밋 틱 종료 시점의 Rapier 진실 상태 = 예측기 락스텝의 초기값.
        let mut pos = v3(world.ball_position());
        let mut vel = v3(world.ball_velocity());
        let mut omega = v3(world.ball_angular_velocity());
        println!(
            "\n=== {}/{index_in_zone}  impact_time={impact_time:.4}  pred_impact=[{:.4} {:.4} {:.4}]\n\
             (락스텝 시작: 커밋 틱 종료. 예측기는 이 시점 상태로 다시 적분한다) ===",
            zone.label(),
            prediction.impact_position.coords.x,
            prediction.impact_position.coords.y,
            prediction.impact_position.coords.z,
        );
        println!(
            "{:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
            "t", "rap_y", "rap_z", "est_z", "dz", "rap_vy", "est_vy", "rap_wx", "est_wx"
        );
        let mut elapsed = 0.0;
        let mut printed_bounce = false;
        for step in 0..MAX_CONTACT_STEPS {
            // 예측기 한 스텝 (Rapier와 같은 dt).
            let prev_est_vz = vel.z;
            let (np, nv, nw) =
                pingpong_bot::estimator::semi_implicit_euler(pos, vel, omega, DT, &physics);
            pos = np;
            vel = nv;
            omega = nw;
            let est_bounced = prev_est_vz < -0.3 && vel.z > 0.05;

            let prev_rap_vz = v3(world.ball_velocity()).z;
            world.step(DT, None);
            elapsed += DT;
            let rap = v3(world.ball_position());
            let rap_v = v3(world.ball_velocity());
            let rap_w = v3(world.ball_angular_velocity());
            let rap_bounced = prev_rap_vz < -0.3 && rap_v.z > 0.05;

            if (est_bounced || rap_bounced) && !printed_bounce {
                println!(
                    "  -- bounce: est={est_bounced} rapier={rap_bounced} @ t={elapsed:.4} \
                     (est vz {prev_est_vz:+.3}->{:+.3}, rapier vz {prev_rap_vz:+.3}->{:+.3})",
                    vel.z, rap_v.z
                );
                printed_bounce = true;
            }
            let near_impact = elapsed > impact_time - 0.006;
            if step % 25 == 0 || near_impact {
                println!(
                    "{elapsed:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>+8.4} {:>8.3} {:>8.3} {:>8.2} {:>8.2}",
                    rap.y, rap.z, pos.z, pos.z - rap.z, rap_v.y, vel.y, rap_w.x, omega.x,
                );
            }
            if elapsed >= impact_time || ball_touches_racket(&world) {
                println!(
                    "  -- 종료 @ t={elapsed:.4}: rapier=[{:.4} {:.4} {:.4}] est=[{:.4} {:.4} {:.4}] \
                     |Δ|={:.4} touch={}",
                    rap.x,
                    rap.y,
                    rap.z,
                    pos.x,
                    pos.y,
                    pos.z,
                    (pos - rap).norm(),
                    ball_touches_racket(&world),
                );
                break;
            }
        }
    }
}

/// 팀리드 후속 실험: `diag_table_restitution::diag_restitution_vs_solver_knobs`가
/// 찾은 `normalized_prediction_distance` 단서가 공-라켓 접촉 타이밍(`d_total`)
/// 에도 같은 효과를 내는지 확인한다. `plane_cross`가 이 하네스에서 항상
/// `None`이라(아래 참고) `d_pred`/`d_geom` 분해는 못 쓰고, `d_total`(= 유일하게
/// 신뢰 가능한 지표, 진단 보고서의 "4.7ms 조기 발동"과 동일 정의)만 비교한다.
///
/// **하네스 결함 발견**: 대표 그리드(Left/Center/Right 각 10샷)에서
/// `plane_cross_secs`가 30/30 전부 `None`이었다 — `world.debug_prediction()`이
/// 반환하는 예측이 실제 커밋된 스윙이 타겟한 평면과 다를 수 있다(여러 후보
/// 평면 중 `plan_best_swing`이 고른 것과 `debug_prediction()`의 최신 캐시가
/// 어긋남). `d_pred`/`d_geom` 성분 분해는 이 버그를 고치기 전까지 근거로 쓰면
/// 안 된다 — 후속 과제로 남긴다. `d_total`은 별도 카운터(`contact_secs`,
/// `planned_secs`)라 이 버그와 무관하고 그대로 신뢰할 수 있다.
#[test]
#[ignore = "진단 전용"]
fn diag_contact_timing_solver_knob_sweep() {
    type Tweak = Box<dyn Fn(&mut SimWorld)>;
    let launch = EvalLaunchParams::default();
    let shots: Vec<(EvalZone, usize)> = Protocol::shot_schedule(EvalMode::Block);

    let variants: Vec<(&str, Tweak)> = vec![
        ("default", Box::new(|_: &mut SimWorld| {})),
        (
            "pred_dist~0",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.normalized_prediction_distance = 1e-6;
            }),
        ),
        (
            "solver_iters=32",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.num_solver_iterations = 32;
            }),
        ),
    ];

    for (label, tweak) in &variants {
        let rows: Vec<TimingRow> = shots
            .iter()
            .enumerate()
            .map(|(i, &(zone, index_in_zone))| {
                let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
                run_shot_tweaked(
                    format!("{}#{}", zone.label(), i + 1),
                    launch.speed_mps,
                    &settings,
                    &tweak,
                )
            })
            .collect();
        let usable: Vec<&TimingRow> = rows
            .iter()
            .filter(|r| r.committed && r.contact_secs.is_some())
            .collect();
        let d_total = |r: &TimingRow| r.contact_secs.unwrap() - r.planned_secs;
        if usable.is_empty() {
            println!("=== {label} ===  usable 샷 없음");
            continue;
        }
        let mean = usable.iter().map(|r| d_total(r)).sum::<f64>() / usable.len() as f64;
        let min = usable
            .iter()
            .map(|r| d_total(r))
            .fold(f64::INFINITY, f64::min);
        let max = usable
            .iter()
            .map(|r| d_total(r))
            .fold(f64::NEG_INFINITY, f64::max);
        println!(
            "=== {label:<18} ===  n={:<3} commit={}/{} mean_d_total={mean:+.5}s min={min:+.5}s max={max:+.5}s",
            usable.len(),
            rows.iter().filter(|r| r.committed).count(),
            rows.len(),
        );
    }
}

/// eval 30샷 격자 — 기본 발사속도.
#[test]
#[ignore = "진단 전용"]
fn diag_contact_timing_eval_grid() {
    let launch = EvalLaunchParams::default();
    let rows: Vec<TimingRow> = Protocol::shot_schedule(EvalMode::Block)
        .into_iter()
        .enumerate()
        .map(|(i, (zone, index_in_zone))| {
            let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
            run_shot(
                format!("{}#{}", zone.label(), i + 1),
                launch.speed_mps,
                &settings,
            )
        })
        .collect();
    print_table(&rows);
}

/// 발사속도 스윕 — 타이밍 오차가 접근속도에 **비례**(기하 스윕)하는지
/// **무관**(적분 드리프트)한지 가르는 결정 실험.
#[test]
#[ignore = "진단 전용"]
fn diag_contact_timing_speed_sweep() {
    let mut rows = Vec::new();
    for speed in [4.5, 5.5, 6.0, 6.5, 7.5, 8.5] {
        let launch = EvalLaunchParams {
            speed_mps: speed,
            ..EvalLaunchParams::default()
        };
        for (zone, index_in_zone) in [
            (EvalZone::Center, 0),
            (EvalZone::Center, 4),
            (EvalZone::Left, 2),
            (EvalZone::Right, 2),
        ] {
            let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
            rows.push(run_shot(
                format!("{}/{}", zone.label(), index_in_zone),
                speed,
                &settings,
            ));
        }
    }
    print_table(&rows);
}
