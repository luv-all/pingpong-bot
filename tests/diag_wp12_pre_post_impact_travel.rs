//! WP12 진단: 실제 커밋 시점에 quintic이 관절 이동을 "타격 전"(pre-impact
//! segment)과 "타격 후"(follow-through)에 각각 얼마나 남겨두는지를 잰다.
//!
//! 배경: `COARSE_TRACK_JOINT_FRACTION`(`src/sim/physics/world.rs`)이 공이
//! 상대 코트에 있는 동안 회전 관절을 예측 임팩트 자세 쪽으로 미리 80%
//! blend한다. WP10(`docs/wp10-coarse-track-per-joint.md`)은 이 상수를
//! 커밋률·달성 세기(`|v_out|/desired`) 축으로만 스윕했고, 실제 "타격
//! 전/후 이동량 비율" — 사용자가 보고한 "칠 때 멈추고 나머지는 친 뒤에
//! 움직인다" 증상의 직접 지표 — 은 한 번도 측정한 적이 없다. 이 진단이 그
//! 공백을 메운다. 상세: `.omc/plans/2026-07-31-pre-impact-freeze-fix.md`.
//!
//! 계측 방법: `SimDebugSnapshot::set_committed_path`가 커밋 시점에 채우는
//! `pre_impact_travel`/`follow_through_travel`(관절별 |Δq|, 계획 경로가
//! 실제로 쓰는 `Trajectory`에서 직접 뽑음 — 재구현 아님)을 커밋 순간(첫
//! `swing_committed()==true`가 되는 스텝) 한 번만 읽는다.
//!
//! 실행:
//!   cargo test --release --test diag_wp12_pre_post_impact_travel -- --ignored --nocapture

use pingpong_bot::defaults;
use pingpong_bot::robot::State as RobotState;
use pingpong_bot::sim::eval::{LaunchParams as EvalLaunchParams, Mode as EvalMode, Protocol, Zone};
use pingpong_bot::sim::launch::{Layout as ShooterLayout, Settings as BallShooterSettings};
use pingpong_bot::sim::physics::SimWorld;

const DT: f64 = 1.0 / 1000.0;
const MAX_STEPS: usize = 4_000;

const JOINT_LABELS: [&str; 4] = ["q0 base yaw", "q1 shoulder", "q2 elbow", "q3 wrist"];

/// `BallShooterSettings::yaw_range_for_lateral_deg`(crate-private)의 재현.
/// `tests/diag_wp10_coarse_track_per_joint.rs`와 동일 — 같은 랜덤 격자를 본다.
fn yaw_range_for_lateral_deg(lateral_offset_m: f64) -> (f64, f64) {
    use pingpong_bot::constants::table;
    let mount_x = ShooterLayout::MOUNT_X + lateral_offset_m;
    let mount_y = ShooterLayout::MOUNT_Y;
    let yaw_for = |target_x: f64| (target_x - mount_x).atan2(mount_y).to_degrees();
    let left = yaw_for(defaults::RANDOM_SHOT_TARGET_PADDING_M);
    let right = yaw_for(table::WIDTH_X - defaults::RANDOM_SHOT_TARGET_PADDING_M);
    return (left.min(right), left.max(right));
}

/// 커밋 순간의 관절별 계측: (타격 전 |Δq|, 타격 후 |Δq|, |임팩트 순간 명령
/// 각속도| / 궤적 전 구간 peak 각속도, 임팩트 knot 공유 명령 각가속도).
#[derive(Debug, Clone, Copy)]
struct JointSample {
    pre_travel: f64,
    post_travel: f64,
    /// 1.0 = 임팩트 순간 이 관절이 궤적 전체의 peak 속도를 그대로 내고 있다
    /// (= "치는 순간에도 계속 움직인다"), 0.0 = 임팩트 순간 정지.
    impact_speed_ratio: f64,
    /// 임팩트 knot 공유 명령 각가속도 [rad/s²] — 예전(항상 0 강제)엔 늘
    /// 0.0이었다. 0이 아닐수록 "타격 순간에도 모터가 계속 일을 하고 있다"는
    /// 직접 증거. `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`.
    impact_acceleration: f64,
}

/// 한 발 실행, 커밋 순간의 관절별 계측을 반환. 커밋되지 않으면 `None`.
/// WP9와 동일하게 매 샷 전 레일을 `default_x()`로 리셋한다.
fn run_shot(settings: &BallShooterSettings) -> Option<Vec<JointSample>> {
    let robot = defaults::robot().expect("robot");
    let mut world = SimWorld::with_physics(robot.clone(), defaults::PhysicsParams::default());
    world.set_use_ground_truth(true);
    if let Some(rail) = robot.arm.rail {
        *world.robot_mut() = RobotState::new(robot.arm.default_joints.clone(), rail.default_x());
    }
    world.shoot_ball(settings);

    let mut was_committed = false;
    for _ in 0..MAX_STEPS {
        world.step(DT, None);
        if world.swing_committed() && !was_committed {
            let snap = world.debug_snap();
            let n = snap.pre_impact_travel.len();
            let samples = (0..n)
                .map(|i| {
                    let peak = snap.peak_joint_speed_overall.get(i).copied().unwrap_or(0.0);
                    let impact = snap.impact_velocity.get(i).copied().unwrap_or(0.0).abs();
                    JointSample {
                        pre_travel: snap.pre_impact_travel[i],
                        post_travel: snap.follow_through_travel[i],
                        impact_speed_ratio: if peak > 1e-9 { impact / peak } else { 0.0 },
                        impact_acceleration: snap.impact_acceleration.get(i).copied().unwrap_or(0.0),
                    }
                })
                .collect();
            return Some(samples);
        }
        was_committed = world.swing_committed();
    }
    return None;
}

#[derive(Debug, Clone, Default)]
struct JointAccum {
    pre_sum: f64,
    post_sum: f64,
    /// 샷별 ratio(=pre/(pre+post))의 합 — 표본 간 total travel 크기 차이에
    /// 결과가 좌우되지 않도록 평균은 샷별 비율의 평균으로 잡는다.
    ratio_sum: f64,
    impact_speed_ratio_sum: f64,
    impact_accel_abs_sum: f64,
    n: usize,
}

impl JointAccum {
    fn push(&mut self, sample: &JointSample) {
        self.pre_sum += sample.pre_travel;
        self.post_sum += sample.post_travel;
        let total = sample.pre_travel + sample.post_travel;
        if total > 1e-9 {
            self.ratio_sum += sample.pre_travel / total;
        }
        self.impact_speed_ratio_sum += sample.impact_speed_ratio;
        self.impact_accel_abs_sum += sample.impact_acceleration.abs();
        self.n += 1;
    }
    fn mean_ratio(&self) -> f64 {
        return self.ratio_sum / self.n.max(1) as f64;
    }
    fn mean_impact_speed_ratio(&self) -> f64 {
        return self.impact_speed_ratio_sum / self.n.max(1) as f64;
    }
    fn mean_pre_deg(&self) -> f64 {
        return (self.pre_sum / self.n.max(1) as f64).to_degrees();
    }
    fn mean_post_deg(&self) -> f64 {
        return (self.post_sum / self.n.max(1) as f64).to_degrees();
    }
    fn mean_impact_accel_abs(&self) -> f64 {
        return self.impact_accel_abs_sum / self.n.max(1) as f64;
    }
}

#[derive(Debug, Clone, Default)]
struct Grid {
    shots: usize,
    committed: usize,
    joints: Vec<JointAccum>,
}

impl Grid {
    fn push(&mut self, result: Option<Vec<JointSample>>) {
        self.shots += 1;
        let Some(samples) = result else {
            return;
        };
        self.committed += 1;
        if self.joints.len() < samples.len() {
            self.joints.resize(samples.len(), JointAccum::default());
        }
        for (i, sample) in samples.iter().enumerate() {
            self.joints[i].push(sample);
        }
    }
    fn print_table(&self, label: &str) {
        println!(
            "\n#### {label} ({}/{} committed)\n",
            self.committed, self.shots
        );
        println!(
            "| 관절 | 타격 전 평균 [deg] | 타격 후 평균 [deg] | 타격 전 비율 (pre/(pre+post)) | 임팩트 순간 속도 / peak 속도 | 임팩트 knot \\|가속도\\| [rad/s²] |"
        );
        println!("|---|---|---|---|---|---|");
        for (i, accum) in self.joints.iter().enumerate() {
            let label = JOINT_LABELS.get(i).copied().unwrap_or("q?");
            println!(
                "| {label} | {:.2} | {:.2} | {:.3} | {:.3} | {:.2} |",
                accum.mean_pre_deg(),
                accum.mean_post_deg(),
                accum.mean_ratio(),
                accum.mean_impact_speed_ratio(),
                accum.mean_impact_accel_abs(),
            );
        }
    }
}

/// `Zone::zone_index`(crate-private)의 재현.
fn zone_index(zone: Zone) -> usize {
    return match zone {
        Zone::Right => 0,
        Zone::Center => 1,
        Zone::Left => 2,
    };
}

/// eval 그리드 지터 시드 — WP1/WP2b/WP10과 같은 값을 써서 같은 30발을 본다.
const EVAL_SEED: u64 = 0x5741_5031; // "WAP1"

fn eval_grid() -> (Grid, [Grid; 3]) {
    use rand::SeedableRng;
    let launch = EvalLaunchParams::default();
    let mut rng = rand::rngs::StdRng::seed_from_u64(EVAL_SEED);
    let mut all = Grid::default();
    let mut by_zone: [Grid; 3] = Default::default();
    for (zone, index_in_zone) in Protocol::shot_schedule(EvalMode::Alternating) {
        let settings =
            Protocol::settings_for_zone_shot_jittered(&launch, zone, index_in_zone, &mut rng);
        let result = run_shot(&settings);
        all.push(result.clone());
        by_zone[zone_index(zone)].push(result);
    }
    return (all, by_zone);
}

/// 랜덤 슈터 5×5 그리드 — WP1/WP2b/WP10과 동일한 lateral×yaw 격자.
fn random_grid() -> Grid {
    let mut grid = Grid::default();
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
            grid.push(run_shot(&settings));
        }
    }
    return grid;
}

/// WP12 계측 — `COARSE_TRACK_JOINT_FRACTION`의 현재 값 기준 타격 전/후
/// 관절 이동량 비율. 상수를 바꿔 재실행해 A/B로 비교한다(§ 플랜 Step 1/2).
#[test]
#[ignore = "진단 전용 — 55샷 시뮬"]
fn diag_wp12_pre_post_impact_travel_ratio() {
    let (eval_all, by_zone) = eval_grid();
    let rnd = random_grid();

    println!("\n### WP12 타격 전/후 관절 이동량 비율\n");
    eval_all.print_table("eval 30 (all)");
    for zone in [Zone::Left, Zone::Center, Zone::Right] {
        by_zone[zone_index(zone)].print_table(&format!("eval {}", zone.label()));
    }
    rnd.print_table("random 5x5");
    println!(
        "\n비율이 낮을수록(0에 가까울수록) 그 관절은 커밋 시점에 이미 임팩트\n\
         자세에 가까워 \"타격 전\" 구간에 남은 이동량이 적다는 뜻 — 사용자가\n\
         보고한 \"칠 때 멈춤\" 증상과 직접 대응한다."
    );
}
