//! 실험: "가운데, 세기 6.5 m/s" 샷에서 로봇이 고른 타점 높이(오버헤드 vs
//! 스쿱)와 실제 성공률의 관계를 측정한다.
//!
//! 배경: 사용자 관찰 — 같은 "가운데, 6.5" 샷인데 어떤 실행은 라켓이 위에서
//! 내려찍듯 스윙하다 실패하고, 어떤 실행은 아래에서 퍼올려 성공한다. 실제
//! 로봇은 최소 6.5 m/s 정도의 샷을 처리해야 하므로, 이 속도에서 "스쿱"
//! 자세가 "오버헤드" 자세보다 실제로 더 잘 통하는지를 정량 확인한다.
//!
//! 실험 A (자연 분포): 기본 `InterceptWindow`로 center+6.5 샷을 셔터
//! 오차 수준의 작은 지터(속도/요/피치)를 주며 반복 실행 — 커밋된 타점의
//! y/z로 오버헤드/스쿱을 사후 분류하고 그룹별 성공률을 비교한다.
//!
//! 실험 B (인과 확인): `InterceptWindow`를 로봇에 가까운 절반(스쿱 전용)과
//! 먼 절반(오버헤드 전용)으로 강제 분할해 같은 샷을 돌려, 그리디 후보
//! 선택기의 자연스러운 선택에 의존하지 않고 두 자세의 성능을 직접 비교한다.
//!
//! 실행:
//!   cargo test --release --test diag_scoop_vs_overhead_6_5 -- --ignored --nocapture

use rand::Rng;
use rand::SeedableRng;
use rand::rngs::StdRng;

use pingpong_bot::defaults;
use pingpong_bot::robot::motion::InterceptWindow;
use pingpong_bot::sim::eval::LiveObserver;
use pingpong_bot::sim::launch::Settings as BallShooterSettings;
use pingpong_bot::sim::physics::SimWorld;

const DT: f64 = 1.0 / 1000.0;
const MAX_STEPS: usize = 4_000;
const SEED: u64 = 0x5343_4F4F_5021; // "SCOOP!"

#[derive(Debug, Clone, Copy, Default)]
struct TrialResult {
    committed: bool,
    impact_y: f64,
    impact_z: f64,
    /// 커밋 **직전** 틱까지 rough 추종이 쫓던 예측의 타점 y — coarse-track이
    /// 최종 커밋 타점과 다른 곳을 미리 쫓고 있었는지 보기 위한 대조값.
    pre_commit_y: f64,
    contact: bool,
    cleared_net: bool,
    returned_in: bool,
    points: u8,
}

fn run_trial(window: InterceptWindow, settings: &BallShooterSettings) -> TrialResult {
    let robot = defaults::robot().expect("robot");
    let mut world = SimWorld::with_physics(robot, defaults::PhysicsParams::default());
    world.set_use_ground_truth(true);
    world.set_intercept_window(window);
    world.shoot_ball(settings);

    let mut observer = LiveObserver::new(&world);
    let mut out = TrialResult {
        impact_y: f64::NAN,
        impact_z: f64::NAN,
        pre_commit_y: f64::NAN,
        ..Default::default()
    };
    let mut was_committed = false;

    for _ in 0..MAX_STEPS {
        if !was_committed {
            if let Some(prediction) = world.debug_prediction() {
                out.pre_commit_y = prediction.impact_position.coords.y;
            }
        }
        world.step(DT, None);
        if world.swing_committed() && !was_committed {
            out.committed = true;
            // 이 틱에 막 커밋됐다 — `debug_prediction()`이 다음 스텝부터는
            // 최신 탄도 마커로 덮어써지므로 지금 잡아야 원래 커밋된 타점이다.
            if let Some(prediction) = world.debug_prediction() {
                out.impact_y = prediction.impact_position.coords.y;
                out.impact_z = prediction.impact_position.coords.z;
            }
        }
        was_committed = was_committed || world.swing_committed();
        if observer.observe(&world) {
            break;
        }
    }

    out.contact = observer.flags.contact;
    out.cleared_net = observer.flags.cleared_net;
    out.returned_in = observer.flags.returned_in;
    out.points = observer.points();
    return out;
}

/// 셔터 오차 수준 지터를 적용한 "가운데, speed_mps" 샷.
fn jittered_center_shot(speed_mps: f64, rng: &mut StdRng) -> BallShooterSettings {
    let speed_jitter = defaults::sim::EVAL_SPEED_JITTER_MPS;
    let yaw_jitter = defaults::sim::EVAL_YAW_JITTER_DEG;
    let pitch_jitter = defaults::sim::EVAL_PITCH_JITTER_DEG;
    return BallShooterSettings {
        speed_mps: speed_mps + rng.gen_range(-speed_jitter..=speed_jitter),
        yaw_deg: rng.gen_range(-yaw_jitter..=yaw_jitter),
        pitch_deg: BallShooterSettings::default().pitch_deg
            + rng.gen_range(-pitch_jitter..=pitch_jitter),
        ..BallShooterSettings::default()
    };
}

#[derive(Debug, Clone, Copy, Default)]
struct Bucket {
    n: usize,
    committed: usize,
    contact: usize,
    cleared_net: usize,
    returned_in: usize,
    points: u32,
    z_sum: f64,
    y_sum: f64,
    z_n: usize,
    mismatch_sum: f64,
}

impl Bucket {
    fn push(&mut self, r: TrialResult) {
        self.n += 1;
        self.committed += usize::from(r.committed);
        self.contact += usize::from(r.contact);
        self.cleared_net += usize::from(r.cleared_net);
        self.returned_in += usize::from(r.returned_in);
        self.points += u32::from(r.points);
        if r.committed {
            self.z_sum += r.impact_z;
            self.y_sum += r.impact_y;
            self.z_n += 1;
            if r.pre_commit_y.is_finite() {
                self.mismatch_sum += (r.pre_commit_y - r.impact_y).abs();
            }
        }
    }
    fn pct(&self, k: usize) -> f64 {
        return 100.0 * k as f64 / self.n.max(1) as f64;
    }
    fn print(&self, label: &str) {
        println!(
            "  {label:<22} n={:<3} commit={:>5.1}% contact={:>5.1}% net-clear={:>5.1}% in-court={:>5.1}% mean-pts={:.2} mean-impact-y={:.3} mean-impact-z={:.3} mean|pre_commit_y-impact_y|={:.3}",
            self.n,
            self.pct(self.committed),
            self.pct(self.contact),
            self.pct(self.cleared_net),
            self.pct(self.returned_in),
            self.points as f64 / self.n.max(1) as f64,
            self.y_sum / self.z_n.max(1) as f64,
            self.z_sum / self.z_n.max(1) as f64,
            self.mismatch_sum / self.z_n.max(1) as f64,
        );
    }
}

const N_TRIALS: usize = 50;
const SPEED_MPS: f64 = 6.5;

#[test]
#[ignore = "실험용 — 오래 걸림"]
fn experiment_a_natural_plane_distribution_at_6_5() {
    let window = InterceptWindow::default();
    println!(
        "\n=== 실험 A: 자연 분포 — center+{SPEED_MPS} m/s, window={:?}, n={N_TRIALS} ===",
        window
    );

    let mut rng = StdRng::seed_from_u64(SEED);
    let mut rows: Vec<TrialResult> = Vec::with_capacity(N_TRIALS);
    for _ in 0..N_TRIALS {
        let settings = jittered_center_shot(SPEED_MPS, &mut rng);
        rows.push(run_trial(window, &settings));
    }

    let committed: Vec<&TrialResult> = rows.iter().filter(|r| r.committed).collect();
    println!(
        "  커밋 {}/{N_TRIALS} — 미커밋 {}건은 분류 불가(스윙 자체를 못 만듦)",
        committed.len(),
        N_TRIALS - committed.len()
    );
    if committed.is_empty() {
        println!("  커밋된 샷이 없어 분류할 수 없음 — 실험 B로 넘어감");
        return;
    }

    let mut ys: Vec<f64> = committed.iter().map(|r| r.impact_y).collect();
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median_y = ys[ys.len() / 2];
    println!(
        "  커밋된 타점 y 분포: min={:.3} p50={:.3} max={:.3} (median으로 분류 기준 삼음)",
        ys[0],
        median_y,
        ys[ys.len() - 1]
    );

    println!(
        "\n  {:>4} {:>8} {:>8} {:>10} {:>9} {:>8} {:>7} {:>10} {:>10} {:>5}",
        "#", "y", "z", "mismatch", "class", "commit", "contact", "net-clear", "in-court", "pts"
    );
    let (mut scoop, mut overhead) = (Bucket::default(), Bucket::default());
    for (i, r) in rows.iter().enumerate() {
        let class = if !r.committed {
            "미커밋"
        } else if r.impact_y <= median_y {
            "SCOOP(가까움/낮음)"
        } else {
            "OVERHEAD(멈/높음)"
        };
        println!(
            "  {:>4} {:>8.3} {:>8.3} {:>10.3} {:>9} {:>8} {:>7} {:>10} {:>10} {:>5}",
            i,
            r.impact_y,
            r.impact_z,
            (r.pre_commit_y - r.impact_y).abs(),
            class,
            r.committed,
            r.contact,
            r.cleared_net,
            r.returned_in,
            r.points
        );
        if r.committed {
            if r.impact_y <= median_y {
                scoop.push(*r);
            } else {
                overhead.push(*r);
            }
        }
    }

    println!("\n  --- 그룹별 집계 (median-y 분류) ---");
    scoop.print("SCOOP (y<=median)");
    overhead.print("OVERHEAD (y>median)");
}

#[test]
#[ignore = "실험용 — 오래 걸림"]
fn experiment_b_forced_scoop_vs_overhead_window_at_6_5() {
    let full = InterceptWindow::default(); // y=[0.08,0.35]
    let scoop_window = InterceptWindow {
        y_min: 0.08,
        y_max: 0.19,
        sample_step: 0.03,
    };
    let overhead_window = InterceptWindow {
        y_min: 0.20,
        y_max: 0.35,
        sample_step: 0.03,
    };

    println!("\n=== 실험 B: 강제 분할 — center+{SPEED_MPS} m/s, n={N_TRIALS} per window ===");
    println!("  scoop_window={:?}", scoop_window);
    println!("  overhead_window={:?}", overhead_window);
    println!("  full_window(baseline)={:?}", full);

    for (label, window) in [
        ("FULL (baseline, 현재 기본값)", full),
        ("SCOOP-ONLY (로봇에 가까운 절반)", scoop_window),
        ("OVERHEAD-ONLY (로봇에서 먼 절반)", overhead_window),
    ] {
        // 세 조건 모두 **같은 시드로 같은 지터 시퀀스**를 재생해 짝지은
        // 비교가 되도록 한다 — 샷 자체의 무작위성이 아니라 window 강제
        // 분할의 효과만 보기 위함.
        let mut rng = StdRng::seed_from_u64(SEED);
        let mut bucket = Bucket::default();
        for _ in 0..N_TRIALS {
            let settings = jittered_center_shot(SPEED_MPS, &mut rng);
            bucket.push(run_trial(window, &settings));
        }
        bucket.print(label);
    }
}
