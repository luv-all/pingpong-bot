//! WP1 진단: 타점 y평면 범위(`InterceptWindow`) 스윕.
//!
//! `y_min × y_max × sample_step` 조합마다 eval 30샷 그리드 + 랜덤 5×5 그리드를
//! 돌려 커밋률·접촉률·달성 `v_r·n`·eval 점수를 측정한다.
//!
//! 실행:
//!   cargo test --release --test diag_wp1_yplane_sweep -- --ignored --nocapture

use std::sync::atomic::{AtomicUsize, Ordering};

use nalgebra::Vector3;

use pingpong_bot::defaults;
use pingpong_bot::robot::motion::InterceptWindow;
use pingpong_bot::sim::eval::{
    LaunchParams as EvalLaunchParams, LiveObserver as LiveShotObserver, Mode as EvalMode,
    Protocol,
};
use pingpong_bot::sim::launch::{Layout as ShooterLayout, Settings as BallShooterSettings};
use pingpong_bot::sim::physics::SimWorld;

const DT: f64 = 1.0 / 1000.0;
const MAX_STEPS: usize = 4_000;

fn v3(v: rapier3d::prelude::Vector) -> Vector3<f64> {
    return Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z));
}

fn ball_touches_racket(world: &SimWorld) -> bool {
    let find = |parent| {
        world
            .collider_set
            .iter()
            .find_map(|(h, c)| (c.parent() == Some(parent)).then_some(h))
    };
    let (Some(ball), Some(racket)) = (find(world.ball_handle), find(world.racket_handle)) else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball, racket)
        .is_some_and(|p| p.has_any_active_contact());
}

/// 라켓 강체의 접촉점 속도 = v_com + ω × (p − com).
fn racket_point_velocity(world: &SimWorld, point: Vector3<f64>) -> Vector3<f64> {
    let body = &world.rigid_body_set[world.racket_handle];
    return v3(body.linvel()) + v3(body.angvel()).cross(&(point - v3(body.center_of_mass())));
}

fn racket_normal(world: &SimWorld) -> Vector3<f64> {
    let (_, rot) = world.racket_pose();
    return v3(rot * rapier3d::prelude::Vector::new(0.0, 0.0, 1.0)).normalize();
}

/// `BallShooterSettings::yaw_range_for_lateral_deg`(crate-private)의 재현 —
/// 마운트에서 로봇쪽 테이블 padding 안쪽을 조준하는 yaw 범위 [deg].
fn yaw_range_for_lateral_deg(lateral_offset_m: f64) -> (f64, f64) {
    use pingpong_bot::constants::table;
    let mount_x = ShooterLayout::MOUNT_X + lateral_offset_m;
    let mount_y = ShooterLayout::MOUNT_Y;
    let yaw_for = |target_x: f64| (target_x - mount_x).atan2(mount_y).to_degrees();
    let left = yaw_for(defaults::RANDOM_SHOT_TARGET_PADDING_M);
    let right = yaw_for(table::WIDTH_X - defaults::RANDOM_SHOT_TARGET_PADDING_M);
    return (left.min(right), left.max(right));
}

#[derive(Debug, Clone, Copy, Default)]
struct ShotResult {
    committed: bool,
    contact: bool,
    points: u8,
    /// 임팩트 직전 라켓 법선속도 v_r·n [m/s] (접촉 시에만 유효).
    vrn: f64,
    /// 임팩트 직전 라켓 속도 크기 |v_r| [m/s] — v_r·n이 작은 게 법선 문제인지
    /// 스윙 자체가 약한 건지 구분하기 위한 대조값.
    vr_mag: f64,
}

fn run_shot(window: InterceptWindow, settings: &BallShooterSettings) -> ShotResult {
    let robot = defaults::robot().expect("robot");
    let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
    world.set_use_ground_truth(true);
    world.set_intercept_window(window);
    world.shoot_ball(settings);

    let mut observer = LiveShotObserver::new(&world);
    let mut out = ShotResult::default();
    let mut was_touching = false;
    // 접촉 **직전** 스텝의 라켓 접근속도/법선 — 접촉이 감지되는 시점엔 이미
    // Rapier가 충격량을 풀어 라켓이 감속한 뒤라, 한 스텝 전 값을 써야
    // "라켓이 공을 때리러 들어온 속도"가 된다.
    let mut prev_vr = Vector3::zeros();
    let mut prev_n = Vector3::zeros();

    for _ in 0..MAX_STEPS {
        let pre_vr = racket_point_velocity(&world, v3(world.ball_position()));
        let pre_n = racket_normal(&world);

        world.step(DT, None);
        if world.swing_committed() {
            out.committed = true;
        }
        let touching = ball_touches_racket(&world);
        if touching && !was_touching && !out.contact {
            out.contact = true;
            out.vrn = prev_vr.dot(&prev_n);
            out.vr_mag = prev_vr.norm();
        }
        was_touching = touching;
        prev_vr = pre_vr;
        prev_n = pre_n;
        if observer.observe(&world) {
            break;
        }
    }
    out.points = observer.points();
    return out;
}

#[derive(Debug, Clone, Copy, Default)]
struct GridStats {
    shots: usize,
    committed: usize,
    contact: usize,
    points: u32,
    vrn_sum: f64,
    vr_mag_sum: f64,
    vrn_n: usize,
}

impl GridStats {
    fn push(&mut self, r: ShotResult) {
        self.shots += 1;
        self.committed += usize::from(r.committed);
        self.contact += usize::from(r.contact);
        self.points += u32::from(r.points);
        if r.contact {
            self.vrn_sum += r.vrn;
            self.vr_mag_sum += r.vr_mag;
            self.vrn_n += 1;
        }
    }
    fn commit_pct(&self) -> f64 {
        return 100.0 * self.committed as f64 / self.shots.max(1) as f64;
    }
    fn contact_pct(&self) -> f64 {
        return 100.0 * self.contact as f64 / self.shots.max(1) as f64;
    }
    fn mean_vrn(&self) -> f64 {
        return self.vrn_sum / self.vrn_n.max(1) as f64;
    }
    fn mean_vr_mag(&self) -> f64 {
        return self.vr_mag_sum / self.vrn_n.max(1) as f64;
    }
}

/// eval 그리드 지터 시드 — 모든 조합이 **같은 30발**을 보게 고정한다.
const EVAL_SEED: u64 = 0x5741_5031; // "WAP1"

/// eval 30샷 그리드.
///
/// **지터를 반드시 켠다.** `settings_for_zone_shot`은 `index_in_zone`을
/// 통째로 버려서(`let _ = index_in_zone;`) 존당 10발이 전부 동일한 샷이
/// 된다 — 실질 표본이 3발뿐이라 커밋률이 1/3 배수로만 움직여 조합을
/// 구분할 해상도가 없다. 고정 시드 지터를 쓰면 30발이 서로 다르면서도
/// 조합 간에는 완전히 동일한 입력이 되어 짝지은 비교가 유지된다.
fn eval_grid(window: InterceptWindow) -> GridStats {
    use rand::SeedableRng;
    let launch = EvalLaunchParams::default();
    let mut rng = rand::rngs::StdRng::seed_from_u64(EVAL_SEED);
    let mut stats = GridStats::default();
    for (zone, index_in_zone) in Protocol::shot_schedule(EvalMode::Alternating) {
        let settings = Protocol::settings_for_zone_shot_jittered(&launch, zone, index_in_zone, &mut rng);
        stats.push(run_shot(window, &settings));
    }
    return stats;
}

/// 랜덤 슈터 5×5 그리드 — lateral 5단계 × 그 lateral의 yaw 범위 5분할.
fn random_grid(window: InterceptWindow) -> GridStats {
    let mut stats = GridStats::default();
    for lateral in [-0.5_f64, -0.25, 0.0, 0.25, 0.5] {
        let (yaw_min, yaw_max) = yaw_range_for_lateral_deg(lateral);
        for k in 0..5 {
            let yaw = yaw_min + (yaw_max - yaw_min) * (k as f64 / 4.0);
            let mut settings = BallShooterSettings {
                lateral_offset_m: lateral,
                yaw_deg: yaw,
                speed_mps: defaults::RANDOM_SHOT_SPEED_MIN_MPS,
                ..BallShooterSettings::default()
            };
            settings.topspin_rad_s = 0.0;
            settings.sidespin_rad_s = 0.0;
            settings.drill_spin_rad_s = 0.0;
            stats.push(run_shot(window, &settings));
        }
    }
    return stats;
}

const Y_MINS: [f64; 3] = [0.05, 0.08, 0.12];
const Y_MAXS: [f64; 3] = [0.25, 0.35, 0.45];
const STEPS: [f64; 3] = [0.02, 0.03, 0.05];

fn combos() -> Vec<InterceptWindow> {
    let mut out = Vec::new();
    for &y_min in &Y_MINS {
        for &y_max in &Y_MAXS {
            for &sample_step in &STEPS {
                out.push(InterceptWindow {
                    y_min,
                    y_max,
                    sample_step,
                });
            }
        }
    }
    return out;
}

fn print_table(rows: &[(InterceptWindow, GridStats, GridStats)]) {
    println!(
        "\n| y_min | y_max | step | planes | eval commit% | eval contact% | eval score/90 | eval v_r·n | eval \\|v_r\\| | rnd commit% | rnd contact% | rnd v_r·n | rnd \\|v_r\\| |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for (w, e, r) in rows {
        println!(
            "| {:.2} | {:.2} | {:.2} | {} | {:.1} | {:.1} | {} | {:.3} | {:.3} | {:.1} | {:.1} | {:.3} | {:.3} |",
            w.y_min,
            w.y_max,
            w.sample_step,
            w.hit_planes().len(),
            e.commit_pct(),
            e.contact_pct(),
            e.points,
            e.mean_vrn(),
            e.mean_vr_mag(),
            r.commit_pct(),
            r.contact_pct(),
            r.mean_vrn(),
            r.mean_vr_mag(),
        );
    }
}

#[test]
#[ignore = "진단 전용 — 오래 걸림"]
fn diag_wp1_yplane_sweep() {
    let all = combos();
    let next = AtomicUsize::new(0);
    let threads = std::thread::available_parallelism()
        .map(|n| n.get().saturating_sub(1).max(1))
        .unwrap_or(4)
        .min(all.len());
    println!("WP1 y평면 스윕: {} 조합, {threads} 스레드", all.len());

    let mut results: Vec<Option<(GridStats, GridStats)>> = vec![None; all.len()];
    std::thread::scope(|scope| {
        let mut handles = Vec::new();
        for _ in 0..threads {
            let next = &next;
            let all = &all;
            handles.push(scope.spawn(move || {
                let mut local = Vec::new();
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= all.len() {
                        break;
                    }
                    let w = all[i];
                    let e = eval_grid(w);
                    let r = random_grid(w);
                    println!(
                        "  [{i:02}] y=[{:.2},{:.2}] step={:.2} planes={} \
                         eval {:.0}%/{:.0}%/{}pt  rnd {:.0}%/{:.0}% vrn={:.2}",
                        w.y_min,
                        w.y_max,
                        w.sample_step,
                        w.hit_planes().len(),
                        e.commit_pct(),
                        e.contact_pct(),
                        e.points,
                        r.commit_pct(),
                        r.contact_pct(),
                        r.mean_vrn(),
                    );
                    local.push((i, e, r));
                }
                return local;
            }));
        }
        for h in handles {
            for (i, e, r) in h.join().expect("worker") {
                results[i] = Some((e, r));
            }
        }
    });

    let rows: Vec<_> = all
        .iter()
        .zip(&results)
        .map(|(w, r)| {
            let (e, rr) = r.expect("result");
            return (*w, e, rr);
        })
        .collect();
    print_table(&rows);

    let default = InterceptWindow::default();
    println!("\n현재 기본값: {default:?}");
}

/// 계측 타당성 검사 — v_r·n이 왜 작은지 샷별로 분해해 본다.
///
/// `|v_racket|`이 크고 `v_r·n`만 작으면 법선/기하 문제, 둘 다 작으면
/// 라켓이 실제로 안 움직인 것(수동 접촉)이다.
#[test]
#[ignore = "진단 전용"]
fn diag_wp1_metric_sanity() {
    let window = InterceptWindow::default();
    let launch = EvalLaunchParams::default();
    println!("{window:?}\n");
    println!("| # | zone | committed | contact | |v_r| | v_r·n | cos(v_r,n) | pts |");
    println!("|---|---|---|---|---|---|---|---|");

    for (i, (zone, index_in_zone)) in Protocol::shot_schedule(EvalMode::Alternating)
        .into_iter()
        .enumerate()
        .take(12)
    {
        let settings = Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let robot = defaults::robot().expect("robot");
        let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
        world.set_use_ground_truth(true);
        world.set_intercept_window(window);
        world.shoot_ball(&settings);

        let mut observer = LiveShotObserver::new(&world);
        let (mut committed, mut contact) = (false, false);
        let (mut vr_mag, mut vrn, mut cos) = (0.0, 0.0, 0.0);
        let mut was_touching = false;
        for _ in 0..MAX_STEPS {
            world.step(DT, None);
            committed |= world.swing_committed();
            let touching = ball_touches_racket(&world);
            if touching && !was_touching && !contact {
                let p = v3(world.ball_position());
                let v = racket_point_velocity(&world, p);
                let n = racket_normal(&world);
                contact = true;
                vr_mag = v.norm();
                vrn = v.dot(&n);
                cos = if vr_mag > 1e-9 { vrn / vr_mag } else { 0.0 };
            }
            was_touching = touching;
            if observer.observe(&world) {
                break;
            }
        }
        println!(
            "| {i} | {:?} | {committed} | {contact} | {vr_mag:.3} | {vrn:.3} | {cos:+.3} | {} |",
            zone,
            observer.points()
        );
    }
}

/// 실제로 **선택된 타점 y**의 분포 — 창 경계가 정말 구속력이 있는지 검사.
///
/// `plan_best_swing`은 후보를 `in_swing_commit_window`(0.08~0.35s)로 거른 뒤
/// **현재 라켓 위치와의 거리**로 정렬한다. 공은 −y로 오므로 y가 클수록
/// time-to-impact가 작다 — 즉 실제 채택 가능한 y 띠는 커밋 시간창이 정하고,
/// `y_min`/`y_max`는 그 띠를 잘라낼 때만 의미가 있다. 접촉 시점 공 y를
/// 찍어 실제 타점 분포가 창 안쪽 어디에 몰리는지 본다.
#[test]
#[ignore = "진단 전용"]
fn diag_wp1_selected_plane_distribution() {
    use rand::SeedableRng;
    let launch = EvalLaunchParams::default();

    for window in [
        InterceptWindow::default(),
        InterceptWindow {
            y_min: 0.05,
            y_max: 0.45,
            sample_step: 0.02,
        },
        InterceptWindow {
            y_min: 0.12,
            y_max: 0.25,
            sample_step: 0.05,
        },
    ] {
        let mut rng = rand::rngs::StdRng::seed_from_u64(EVAL_SEED);
        let mut impact_ys: Vec<f64> = Vec::new();
        for (zone, index_in_zone) in Protocol::shot_schedule(EvalMode::Alternating) {
            let settings = Protocol::settings_for_zone_shot_jittered(&launch, zone, index_in_zone, &mut rng);
            let robot = defaults::robot().expect("robot");
            let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
            world.set_use_ground_truth(true);
            world.set_intercept_window(window);
            world.shoot_ball(&settings);
            let mut observer = LiveShotObserver::new(&world);
            let mut was_touching = false;
            for _ in 0..MAX_STEPS {
                world.step(DT, None);
                let touching = ball_touches_racket(&world);
                if touching && !was_touching {
                    impact_ys.push(f64::from(world.ball_position().y));
                    break;
                }
                was_touching = touching;
                if observer.observe(&world) {
                    break;
                }
            }
        }
        impact_ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let planes = window.hit_planes();
        println!(
            "\n창 y=[{:.2},{:.2}] step={:.2} → 평면 {}개 (y={:.2}..{:.2})",
            window.y_min,
            window.y_max,
            window.sample_step,
            planes.len(),
            planes.first().map(|p| p.y).unwrap_or(f64::NAN),
            planes.last().map(|p| p.y).unwrap_or(f64::NAN),
        );
        if impact_ys.is_empty() {
            println!("  접촉 없음");
            continue;
        }
        let n = impact_ys.len();
        let mean = impact_ys.iter().sum::<f64>() / n as f64;
        println!(
            "  접촉 {n}발 | 실제 타점 y: min={:.3} p50={:.3} max={:.3} mean={:.3}",
            impact_ys[0],
            impact_ys[n / 2],
            impact_ys[n - 1],
            mean,
        );
    }
}

/// 단일 조합 타이밍 — 전체 스윕 비용 추정용.
#[test]
#[ignore = "진단 전용"]
fn diag_wp1_single_combo_timing() {
    let w = InterceptWindow::default();
    let e = eval_grid(w);
    let r = random_grid(w);
    println!("{w:?}");
    println!(
        "eval: commit {:.1}% contact {:.1}% score {}/90 v_r·n {:.3}",
        e.commit_pct(),
        e.contact_pct(),
        e.points,
        e.mean_vrn()
    );
    println!(
        "rnd:  commit {:.1}% contact {:.1}% v_r·n {:.3}",
        r.commit_pct(),
        r.contact_pct(),
        r.mean_vrn()
    );
}
