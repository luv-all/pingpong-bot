//! WP6 진단: Rapier 테이블–공 접촉이 **실제로 실현하는** 반발계수 측정.
//!
//! `PhysicsParams::restitution = 0.88`을 테이블·공 collider 양쪽에 넣었으므로
//! Rapier 기본 Average combine이면 유효 e = 0.88이어야 한다. 그런데 커밋 예측
//! 락스텝 계측(`diag_contact_timing::diag_predictor_vs_rapier_divergence`)에서
//! Rapier는 vz −2.940 → +2.349 (e ≈ 0.80)를 냈고 예측기 커널은 +2.596
//! (e = 0.88)을 냈다. 이 차이가 접촉 타이밍 불일치(RC-3)의 단일 근원이다.
//!
//! 여기서는 (1) 유효 e가 상수비인지 속도 의존인지, (2) 산포가 서브틱 접촉
//! 위상 아티팩트인지, (3) Rapier 솔버 노브로 설정값에 맞출 수 있는지를 가른다.

use pingpong_bot::defaults;
use pingpong_bot::sim::SimWorld;

const DT: f64 = 1.0 / 1000.0;

fn fresh_world() -> SimWorld {
    let mut world = SimWorld::with_physics(
        defaults::robot().expect("robot"),
        defaults::PhysicsParams::default(),
    );
    // 로봇이 개입하지 않도록 자동 스윙을 끈다.
    world.set_use_ground_truth(false);
    return world;
}

/// 공을 `drop_height_m` 위에서 접선속도 `tangential_vy`로 놓는다.
fn launch_over_table(world: &mut SimWorld, drop_height_m: f64, tangential_vy: f64) {
    let table_z = pingpong_bot::constants::table::SURFACE_Z;
    let radius = pingpong_bot::constants::ball::RADIUS;
    // 접선속도가 있어도 바운스가 테이블 위에서 일어나도록 +y 끝에서 출발.
    world.launch_ball_at(
        [
            (pingpong_bot::constants::table::WIDTH_X * 0.5) as f32,
            (pingpong_bot::constants::table::LENGTH_Y - 0.2) as f32,
            (table_z + radius + drop_height_m) as f32,
        ],
        [0.0, tangential_vy as f32, 0.0],
        [0.0, 0.0, 0.0],
    );
}

/// 법선속도 부호 반전 직전/직후 `(v_in_z, v_out_z)`.
fn first_bounce(world: &mut SimWorld, step_dt: f64, max_steps: usize) -> Option<(f64, f64)> {
    let mut prev_vz = 0.0;
    for _ in 0..max_steps {
        world.step(step_dt, None);
        let vz = f64::from(world.ball_velocity().z);
        if prev_vz < -0.3 && vz > 0.05 {
            return Some((prev_vz, vz));
        }
        prev_vz = vz;
    }
    return None;
}

fn measure_bounce(drop_height_m: f64, tangential_vy: f64) -> Option<(f64, f64)> {
    let mut world = fresh_world();
    launch_over_table(&mut world, drop_height_m, tangential_vy);
    return first_bounce(&mut world, DT, 3_000);
}

fn summarize(label: &str, es: &[f64]) {
    if es.is_empty() {
        println!("  {label:<28} 측정 실패");
        return;
    }
    let n = es.len() as f64;
    let mean = es.iter().sum::<f64>() / n;
    let min = es.iter().copied().fold(f64::INFINITY, f64::min);
    let max = es.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    println!(
        "  {label:<28} n={:<3} mean={mean:.4} min={min:.4} max={max:.4} spread={:.4}",
        es.len(),
        max - min
    );
}

#[test]
#[ignore = "진단 전용"]
fn diag_rapier_effective_table_restitution() {
    let configured = defaults::PhysicsParams::default().restitution;
    println!("configured restitution (table & ball collider) = {configured}");
    println!(
        "{:>8} {:>8} {:>9} {:>9} {:>9}",
        "drop[m]", "v_t[m/s]", "v_in_z", "v_out_z", "e_eff"
    );
    let mut ratios = Vec::new();
    for drop in [0.30, 0.45, 0.60] {
        for v_t in [0.0, -3.0, -4.5, -6.0] {
            let Some((v_in, v_out)) = measure_bounce(drop, v_t) else {
                println!("{drop:>8.2} {v_t:>8.1}   바운스 감지 실패");
                continue;
            };
            let e = v_out / -v_in;
            ratios.push(e);
            println!("{drop:>8.2} {v_t:>8.1} {v_in:>9.3} {v_out:>9.3} {e:>9.4}");
        }
    }
    println!();
    summarize("all", &ratios);
    println!("(configured {configured})");
}

/// 유효 e의 산포가 **접근 속도 의존**인지 **서브틱 위상 아티팩트**인지 가른다.
///
/// 낙하 높이를 0.1 mm 단위로 흔들면 접근 속도는 거의 그대로인데 1 ms 격자 대비
/// 접촉 위상만 달라진다. 여기서 e가 크게 흔들리면 산포의 원인은 물리(속도 의존
/// 반발)가 아니라 이산 시간 접촉 해석이다 — 그러면 예측기를 **단일 상수**로
/// 정합하는 것이 최선이고, 남는 산포는 시뮬을 고치지 않는 한 줄일 수 없다.
#[test]
#[ignore = "진단 전용"]
fn diag_effective_restitution_subtick_phase() {
    println!(
        "{:>10} {:>9} {:>9} {:>9}",
        "drop[m]", "v_in_z", "v_out_z", "e_eff"
    );
    let mut v_ins = Vec::new();
    let mut es = Vec::new();
    for i in 0..12 {
        let drop = 0.45 + f64::from(i) * 0.0001;
        let Some((v_in, v_out)) = measure_bounce(drop, -4.5) else {
            continue;
        };
        let e = v_out / -v_in;
        v_ins.push(v_in);
        es.push(e);
        println!("{drop:>10.4} {v_in:>9.4} {v_out:>9.4} {e:>9.4}");
    }
    if es.is_empty() {
        return;
    }
    let v_spread = v_ins.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - v_ins.iter().copied().fold(f64::INFINITY, f64::min);
    println!("\n접근속도 산포={v_spread:.5} m/s 인데:");
    summarize("subtick phase sweep", &es);
}

/// Rapier 쪽 노브로 유효 e를 설정값(0.88)에 맞출 수 있는지 — 맞출 수 있으면
/// 예측기에 보정상수를 넣는 대신 **시뮬을 고치는** 쪽이 물리적으로 옳다.
#[test]
#[ignore = "진단 전용"]
fn diag_restitution_vs_solver_knobs() {
    type Tweak = Box<dyn Fn(&mut SimWorld)>;
    let configured = defaults::PhysicsParams::default().restitution;
    let variants: Vec<(&str, Tweak)> = vec![
        ("default", Box::new(|_: &mut SimWorld| {})),
        (
            "ccd=off",
            Box::new(|w: &mut SimWorld| {
                let handle = w.ball_handle;
                if let Some(body) = w.rigid_body_set.get_mut(handle) {
                    body.enable_ccd(false);
                }
            }),
        ),
        (
            "pred_dist~0",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.normalized_prediction_distance = 1e-6;
            }),
        ),
        (
            "pgs_iters=8",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.num_internal_pgs_iterations = 8;
            }),
        ),
        (
            "stiff contact nf=1e4 dr=1",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.contact_softness.natural_frequency = 1.0e4;
                w.integration_parameters.contact_softness.damping_ratio = 1.0;
            }),
        ),
        (
            "allowed_err~0",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.normalized_allowed_linear_error = 1e-6;
            }),
        ),
        (
            "solver_iters=32",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.num_solver_iterations = 32;
            }),
        ),
        (
            "dt=0.25ms",
            Box::new(|w: &mut SimWorld| {
                w.integration_parameters.dt = 0.25 / 1000.0;
            }),
        ),
    ];

    println!("configured e = {configured}");
    for (label, tweak) in &variants {
        let mut es = Vec::new();
        for i in 0..10 {
            let drop = 0.45 + f64::from(i) * 0.0001;
            let mut world = fresh_world();
            launch_over_table(&mut world, drop, -4.5);
            tweak(&mut world);
            let step_dt = f64::from(world.integration_parameters.dt);
            if let Some((v_in, v_out)) = first_bounce(&mut world, step_dt, 12_000) {
                es.push(v_out / -v_in);
            }
        }
        summarize(label, &es);
    }
}
