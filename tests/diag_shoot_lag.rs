//! 임시 진단: shoot 직후 GUI 스터터의 원인을 물리 틱 비용으로 계측한다.
//!
//! `src/sim/session/run.rs`의 물리 스레드 루프를 헤드리스로 재현한다
//! (physics_hz = 1000 → 틱당 예산 1.0 ms). 각 틱마다 다음을 측정한다.
//!   1. `SimWorld::step` 전체 wall 시간
//!   2. `try_auto_swing` 전체 / 그중 marker·predictions 탄도 스캔
//!   3. Rapier `physics_pipeline.step`
//!   4. `refresh_debug_snap` (RNEA 토크 계산 포함)
//!
//! 실행: `cargo test --test diag_shoot_lag -- --nocapture`

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use pingpong_bot::defaults;
use pingpong_bot::sim::SimWorld;
use pingpong_bot::sim::gui::{SimViewer, WORLD_LOCK_WAIT};
use pingpong_bot::sim::launch;
use pingpong_bot::sim::physics;
use pingpong_bot::sim::physics::world::SimStepInput;

const PHYSICS_DT: f64 = 1.0 / 1000.0;
const TICK_BUDGET_US: f64 = 1000.0;

#[derive(Default)]
struct Stats {
    n: usize,
    sum: f64,
    max: f64,
}

impl Stats {
    fn push(&mut self, v: f64) {
        self.n += 1;
        self.sum += v;
        if v > self.max {
            self.max = v;
        }
    }
    fn mean(&self) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        return self.sum / self.n as f64;
    }
}

/// 한 틱의 계측 스냅샷 [us].
#[derive(Clone, Copy, Default)]
struct Tick {
    t_ms: f64,
    step: f64,
    auto_swing: f64,
    marker: f64,
    predictions: f64,
    rapier: f64,
    debug_snap: f64,
}

fn sample(world: &SimWorld, t_ms: f64, step_us: f64) -> Tick {
    return Tick {
        t_ms,
        step: step_us,
        auto_swing: world.diag_auto_swing_secs * 1e6,
        marker: world.diag_marker_secs * 1e6,
        predictions: world.diag_predictions_secs * 1e6,
        rapier: world.diag_rapier_secs * 1e6,
        debug_snap: world.diag_debug_snap_secs * 1e6,
    };
}

#[test]
fn diag_shoot_lag_tick_cost() {
    let mut world = SimWorld::new(defaults::primitive_4dof().expect("4dof"));
    world.set_use_ground_truth(true);
    let shooter = launch::Settings::default();
    world.sync_shooter_pose(&shooter);

    // (A) 기준선: 공이 주차된(=idle) 상태의 틱 비용.
    let mut idle = (Stats::default(), Stats::default(), Stats::default());
    for _ in 0..2_000 {
        let t = std::time::Instant::now();
        world.step(PHYSICS_DT, None);
        idle.0.push(t.elapsed().as_secs_f64() * 1e6);
        idle.1.push(world.diag_rapier_secs * 1e6);
        idle.2.push(world.diag_debug_snap_secs * 1e6);
    }

    // (B) 발사 후: 비행 구간의 틱 비용 + 단계별 소요.
    world.shoot_ball(&shooter);
    let mut step = Stats::default();
    let mut auto_swing = Stats::default();
    let mut marker = Stats::default();
    let mut predictions = Stats::default();
    let mut rapier = Stats::default();
    let mut debug_snap = Stats::default();
    let mut over_budget = 0usize;
    let mut precommit = Stats::default();
    let mut worst: Vec<Tick> = Vec::new();

    for i in 0..3_000 {
        if world.ball_state != physics::BallState::InFlight {
            break;
        }
        let t = std::time::Instant::now();
        world.step(PHYSICS_DT, None);
        let step_us = t.elapsed().as_secs_f64() * 1e6;
        let tick = sample(&world, i as f64 * PHYSICS_DT * 1e3, step_us);

        step.push(tick.step);
        auto_swing.push(tick.auto_swing);
        marker.push(tick.marker);
        predictions.push(tick.predictions);
        rapier.push(tick.rapier);
        debug_snap.push(tick.debug_snap);
        if tick.step > TICK_BUDGET_US {
            over_budget += 1;
        }
        // predictions 스캔이 도는 구간(= 아직 commit/abandon 전)만 별도 집계.
        if tick.predictions > 0.0 {
            precommit.push(tick.step);
        }
        worst.push(tick);
    }

    worst.sort_by(|a, b| b.step.total_cmp(&a.step));
    worst.truncate(8);

    println!(
        "\n=== diag: shoot-lag 틱 비용 (physics_dt={PHYSICS_DT:.4}s, 틱 예산 {TICK_BUDGET_US:.0}us) ==="
    );
    println!(
        "[idle/parked] step mean={:.1}us max={:.1}us | rapier mean={:.1}us | debug_snap mean={:.1}us",
        idle.0.mean(),
        idle.0.max,
        idle.1.mean(),
        idle.2.mean()
    );
    println!(
        "[in-flight]   step mean={:.1}us max={:.1}us  (n={}, 예산 초과 틱 {}회)",
        step.mean(),
        step.max,
        step.n,
        over_budget
    );
    println!(
        "[pre-commit]  step mean={:.1}us max={:.1}us  (n={})",
        precommit.mean(),
        precommit.max,
        precommit.n
    );
    println!("--- in-flight 단계별 [us] ---");
    for (name, s) in [
        ("try_auto_swing(전체)", &auto_swing),
        ("  ├ marker 스캔", &marker),
        ("  └ predictions 스캔", &predictions),
        ("rapier step", &rapier),
        ("refresh_debug_snap", &debug_snap),
    ] {
        println!("{name:<22} mean={:>8.1}  max={:>9.1}", s.mean(), s.max);
    }
    println!("--- 최악 틱 8개 [us] ---");
    for t in &worst {
        println!(
            "t={:>7.1}ms step={:>8.1} swing={:>8.1} (marker={:>6.1} pred={:>6.1}) rapier={:>8.1} snap={:>7.1}",
            t.t_ms, t.step, t.auto_swing, t.marker, t.predictions, t.rapier, t.debug_snap
        );
    }
    println!();
}

// ---------------------------------------------------------------------------
// 렌더 스레드 stale-frame judder 계측
// ---------------------------------------------------------------------------

/// 렌더 스레드가 월드 스냅샷을 잡는 방식.
#[derive(Clone, Copy)]
enum Acquire {
    /// 수정 전 `viewer.rs` 동작 — 한 번 시도하고 실패하면 즉시 포기.
    TryOnce,
    /// 수정 후 — 프로덕션 코드(`gui::lock_world_for_frame`)를 그대로 호출한다.
    /// 테스트가 구현과 따로 놀지 않도록 복사본이 아니라 실제 함수를 쓴다.
    Production,
    /// 상한 길이별 비교용 (프로덕션 함수와 같은 알고리즘, 예산만 다름).
    BoundedWait(Duration),
}

impl Acquire {
    fn label(self) -> String {
        return match self {
            Acquire::TryOnce => "try_lock (수정 전)".to_string(),
            Acquire::Production => format!(
                "lock_world_for_frame ({}us, 수정 후)",
                WORLD_LOCK_WAIT.as_micros()
            ),
            Acquire::BoundedWait(d) => format!("bounded wait {}us", d.as_micros()),
        };
    }

    fn get(self, world: &Mutex<SimWorld>) -> Option<std::sync::MutexGuard<'_, SimWorld>> {
        match self {
            Acquire::TryOnce => return world.try_lock().ok(),
            Acquire::Production => return SimViewer::lock_world_for_frame(world),
            Acquire::BoundedWait(budget) => {
                let deadline = Instant::now() + budget;
                loop {
                    if let Ok(guard) = world.try_lock() {
                        return Some(guard);
                    }
                    if Instant::now() >= deadline {
                        return None;
                    }
                    std::thread::yield_now();
                }
            }
        }
    }
}

/// 한 구간(주차 또는 비행)의 렌더측 계측 결과.
#[derive(Default)]
struct RenderSample {
    frames: usize,
    miss: usize,
    /// 연속 miss 런 길이 목록 (예: [1,1,3,2] = 3프레임 연속 정체가 한 번).
    runs: Vec<usize>,
    /// 렌더가 실제로 그린 공 위치가 직전 프레임과 같았던 프레임 수.
    frozen: usize,
    /// 렌더가 그린 공 위치의 프레임 간 이동 [m] — 정체 후 점프 크기 확인용.
    max_jump_m: f64,
    /// 진짜 공이 움직이고 있던 프레임 수 (frozen 판정 분모).
    moving: usize,
}

impl RenderSample {
    fn miss_rate(&self) -> f64 {
        return 100.0 * self.miss as f64 / self.frames.max(1) as f64;
    }
    fn max_run(&self) -> usize {
        return self.runs.iter().copied().max().unwrap_or(0);
    }
    fn mean_run(&self) -> f64 {
        if self.runs.is_empty() {
            return 0.0;
        }
        return self.runs.iter().sum::<usize>() as f64 / self.runs.len() as f64;
    }
    fn runs_at_least(&self, n: usize) -> usize {
        return self.runs.iter().filter(|&&r| r >= n).count();
    }
    /// 가장 긴 정체가 화면상 몇 ms 동안 공이 멈춰 보였는지.
    fn worst_freeze_ms(&self) -> f64 {
        return self.max_run() as f64 * 16.667;
    }
}

/// 60fps 렌더 루프 흉내 — 프레임마다 월드를 잡아 공 위치를 읽고,
/// 못 잡으면 **직전 프레임 값을 그대로 다시 그린다** (= 화면상 정체).
fn sample_render(world: &Mutex<SimWorld>, frames: usize, mode: Acquire) -> RenderSample {
    const FRAME_DT: Duration = Duration::from_micros(16_667);

    let mut out = RenderSample::default();
    let mut run = 0usize;
    let mut drawn: Option<[f32; 3]> = None;

    for _ in 0..frames {
        let frame_start = Instant::now();
        out.frames += 1;

        match mode.get(world) {
            Some(guard) => {
                if run > 0 {
                    out.runs.push(run);
                    run = 0;
                }
                // 실제 뷰어가 락을 잡고 하는 일(sync_scene_dynamics + StatusSnapshot)에
                // 해당하는 최소 읽기.
                let p = guard.ball_position();
                let v = guard.ball_velocity();
                let _joints = guard.robot().joints().values.clone();
                let _ = guard.debug_snap().commit_phase;
                drop(guard);

                let now = [p.x, p.y, p.z];
                if f64::from(v.length()) > 0.05 {
                    out.moving += 1;
                }
                if let Some(prev) = drawn {
                    let d = f64::from(
                        ((now[0] - prev[0]).powi(2)
                            + (now[1] - prev[1]).powi(2)
                            + (now[2] - prev[2]).powi(2))
                        .sqrt(),
                    );
                    if d > out.max_jump_m {
                        out.max_jump_m = d;
                    }
                }
                drawn = Some(now);
            }
            None => {
                out.miss += 1;
                run += 1;
                // 락 실패 = 직전 프레임 값 재사용 → 화면상 공이 그 자리에 멈춘다.
                if drawn.is_some() {
                    out.frozen += 1;
                    out.moving += 1;
                }
            }
        }

        let spent = frame_start.elapsed();
        if spent < FRAME_DT {
            std::thread::sleep(FRAME_DT - spent);
        }
    }
    if run > 0 {
        out.runs.push(run);
    }
    return out;
}

/// 물리 스레드(`src/sim/session/run.rs:125-189`)를 재현해 띄우고,
/// 주차/비행 각 구간의 렌더측 정체를 계측한다.
///
/// `batch_lock=true`면 catch-up 배치 전체를 락 1회로 처리한다 (수정안 1).
fn run_judder_probe(mode: Acquire, batch_lock: bool) -> (RenderSample, RenderSample, f64, u64) {
    const PHYSICS_SLEEP: Duration = Duration::from_micros(500);
    const SAMPLE_FRAMES: usize = 120;

    let world = Arc::new(Mutex::new(SimWorld::new(
        defaults::primitive_4dof().expect("4dof"),
    )));
    world.lock().expect("world").set_use_ground_truth(true);
    let shutdown = Arc::new(AtomicBool::new(false));
    let shoot_flag = Arc::new(AtomicBool::new(false));

    let physics_world = Arc::clone(&world);
    let physics_shutdown = Arc::clone(&shutdown);
    let physics_shoot = Arc::clone(&shoot_flag);
    let physics = std::thread::spawn(move || {
        let shooter = launch::Settings::default();
        let mut last_wall = Instant::now();
        let mut sim_debt = 0.0_f64;
        let mut total_steps = 0u64;
        let mut lock_held = Duration::ZERO;

        while !physics_shutdown.load(Ordering::Acquire) {
            let now = Instant::now();
            let wall_dt = now
                .saturating_duration_since(last_wall)
                .as_secs_f64()
                .min(0.05);
            last_wall = now;
            sim_debt += wall_dt;

            let max_catchup = 8u32;
            let mut steps = 0u32;
            while sim_debt >= PHYSICS_DT && steps < max_catchup {
                sim_debt -= PHYSICS_DT;
                steps += 1;
            }

            if steps > 0 {
                let held = Instant::now();
                if batch_lock {
                    // 수정안 1: 배치 전체를 락 1회로.
                    let mut w = physics_world.lock().expect("sim 월드");
                    for _ in 0..steps {
                        let shoot = physics_shoot.swap(false, Ordering::AcqRel);
                        w.step(
                            PHYSICS_DT,
                            Some(SimStepInput {
                                shooter: &shooter,
                                shoot,
                                park: false,
                            }),
                        );
                    }
                    drop(w);
                } else {
                    // 현행: 스텝마다 락을 잡았다 놓는다.
                    for _ in 0..steps {
                        let shoot = physics_shoot.swap(false, Ordering::AcqRel);
                        let mut w = physics_world.lock().expect("sim 월드");
                        w.step(
                            PHYSICS_DT,
                            Some(SimStepInput {
                                shooter: &shooter,
                                shoot,
                                park: false,
                            }),
                        );
                        drop(w);
                    }
                }
                lock_held += held.elapsed();
                total_steps += u64::from(steps);
            }

            if sim_debt > PHYSICS_DT * f64::from(max_catchup) {
                sim_debt = PHYSICS_DT * f64::from(max_catchup);
            }
            std::thread::sleep(PHYSICS_SLEEP);
        }
        return (total_steps, lock_held);
    });

    let parked = sample_render(&world, SAMPLE_FRAMES, mode);
    shoot_flag.store(true, Ordering::Release);
    let flight = sample_render(&world, SAMPLE_FRAMES, mode);

    shutdown.store(true, Ordering::Release);
    let (steps, lock_held) = physics.join().expect("물리 스레드");
    let secs = 2.0 * SAMPLE_FRAMES as f64 * 16.667e-3;
    return (
        parked,
        flight,
        100.0 * lock_held.as_secs_f64() / secs,
        steps,
    );
}

fn report(title: &str, parked: &RenderSample, flight: &RenderSample, duty: f64, steps: u64) {
    println!("\n=== {title} ===");
    println!(
        "물리 스레드 월드락 점유율 {duty:.1}% (총 {steps} step, 60fps 프레임 {}개씩 관측)",
        parked.frames
    );
    for (name, s) in [("parked", parked), ("flight", flight)] {
        println!(
            "[{name:<6}] miss {:>3}/{:<3} ({:>4.1}%) | 연속정체 런: 개수={:<3} 평균={:.2} 최대={:<2} (>=2:{:<3} >=3:{:<3} >=5:{:<3}) | 최악 정체 {:.0}ms | 정체프레임 {}/{} | 최대점프 {:.1}cm",
            s.miss,
            s.frames,
            s.miss_rate(),
            s.runs.len(),
            s.mean_run(),
            s.max_run(),
            s.runs_at_least(2),
            s.runs_at_least(3),
            s.runs_at_least(5),
            s.worst_freeze_ms(),
            s.frozen,
            s.moving,
            s.max_jump_m * 100.0,
        );
    }
}

/// 수정 전/후 judder 비교 — 단순 실패율이 아니라 **연속 정체 런 길이**를 잰다.
///
/// 연속 miss 런 N개 = 공이 N/60초 동안 화면에서 멈췄다가 그만큼 순간이동한다.
#[test]
fn diag_render_stale_frame_judder_baseline() {
    let (before_p, before_f, duty_b, steps_b) = run_judder_probe(Acquire::TryOnce, false);
    report(
        "diag: 수정 전 (try_lock 1회 시도 후 포기)",
        &before_p,
        &before_f,
        duty_b,
        steps_b,
    );

    let (after_p, after_f, duty_a, steps_a) = run_judder_probe(Acquire::Production, false);
    report(
        "diag: 수정 후 (gui::lock_world_for_frame — 프로덕션 코드)",
        &after_p,
        &after_f,
        duty_a,
        steps_a,
    );

    println!(
        "\n>>> 비행 중 정체 프레임 {:.1}% → {:.1}% | 최악 연속 정체 {:.0}ms → {:.0}ms | 최대 프레임 점프 {:.1}cm → {:.1}cm\n",
        before_f.miss_rate(),
        after_f.miss_rate(),
        before_f.worst_freeze_ms(),
        after_f.worst_freeze_ms(),
        before_f.max_jump_m * 100.0,
        after_f.max_jump_m * 100.0,
    );

    // 회귀 가드: 수정 후에는 비행 중 3프레임(50ms) 이상 연속 정체가 없어야 한다.
    assert!(
        after_f.runs_at_least(3) == 0,
        "수정 후에도 비행 중 3프레임 이상 연속 정체가 {}회 남음 (최대 런 {})",
        after_f.runs_at_least(3),
        after_f.max_run()
    );
}

/// 후보 수정안 비교 — 어느 쪽이 연속 정체 런을 실제로 줄이는가.
#[test]
fn diag_render_stale_frame_judder_candidates() {
    let cases: [(Acquire, bool); 4] = [
        (Acquire::TryOnce, false),
        (Acquire::TryOnce, true),
        (Acquire::BoundedWait(Duration::from_micros(500)), false),
        (Acquire::Production, false),
    ];
    for (mode, batch) in cases {
        let (parked, flight, duty, steps) = run_judder_probe(mode, batch);
        let title = format!(
            "{} + {}",
            mode.label(),
            if batch {
                "배치 락 (수정안 1)"
            } else {
                "스텝별 락 (현행)"
            }
        );
        report(&title, &parked, &flight, duty, steps);
    }
    println!();
}
