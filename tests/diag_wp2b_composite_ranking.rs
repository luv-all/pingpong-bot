//! WP2b 진단: 후보 랭킹(거리 단일기준 → 치기쉬움+임팩트세기 복합) A/B 계측.
//!
//! `plan_best_swing`의 후보 정렬 기준만 바꾸는 변경이라, A/B는 **같은 하네스를
//! 코드 변경 전/후로 두 번 돌려** 짝지은 비교를 만든다(런타임 토글을 프로덕션
//! 코드에 넣지 않는다 — 랭킹은 계획 경로의 핫패스다).
//!
//! 계측 항목(§WP2b 수용 기준):
//!   - 존별 커밋률·접촉률
//!   - **커밋된 샷 중** `bounced_own_half` / `double_hit` 발생률
//!     (`Flags::score()`가 이 둘로 점수를 1점에 캡핑한다 —
//!      `src/sim/eval/flags.rs:23-37`. weak-return의 직접 증상.)
//!   - 달성 `|v_r·n|`과 **그 시점 기하에서 필요한** `v_r·n` 대비 달성률
//!   - eval 점수 합
//!
//! 실행:
//!   cargo test --release --test diag_wp2b_composite_ranking -- --ignored --nocapture

use nalgebra::Vector3;

use pingpong_bot::defaults;
use pingpong_bot::robot::State as RobotState;
use pingpong_bot::robot::motion::Impact;
use pingpong_bot::sim::eval::{
    LaunchParams as EvalLaunchParams, LiveObserver, Mode as EvalMode, Protocol, Zone,
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

/// `BallShooterSettings::yaw_range_for_lateral_deg`(crate-private)의 재현.
/// `tests/diag_wp1_yplane_sweep.rs`와 동일 — 랜덤 그리드를 짝지어 비교하기 위해.
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
    bounced_own_half: bool,
    double_hit: bool,
    cleared_net: bool,
    returned_in: bool,
    /// 임팩트 **직전** 스텝의 라켓 법선속도 v_r·n [m/s].
    vrn: f64,
    /// 같은 시점의 |v_r|.
    vr_mag: f64,
    /// 그 시점 기하(측정 v_in·측정 법선)에서 rally 리턴에 **필요한** v_r·n.
    vrn_required: f64,
    /// 접촉 직후 실측 |v_out| — eval 점수가 반칙으로 1점에 캡핑돼 판별력이
    /// 없으므로(`Flags::score()`), 리턴 세기 자체를 직접 재는 대조 지표다.
    v_out_actual: f64,
    /// 같은 임팩트점에서 플래너가 원한 |v_out|.
    v_out_desired: f64,
    /// 플래너가 **원한** v_out이 그 임팩트점에서 네트를 직접 넘는가.
    /// false면 병목이 스윙 세기가 아니라 조준/타점 모델 자체에 있다.
    desired_clears_net: bool,
    /// 실측 v_out이 네트를 직접 넘는가.
    actual_clears_net: bool,
    /// 실측 v_out을 **방향은 그대로 두고** desired 크기로 늘리면 네트를
    /// 넘는가 — "너무 느림"(true)과 "방향이 틀림"(false)을 가른다.
    rescaled_clears_net: bool,
}

/// 한 발 실행.
///
/// WP9와 동일하게 매 샷 전에 레일을 `default_x()`로 리셋한다 —
/// `Arm::initial_state()`의 `home_x()`(레일 끝단)에서 시작하면 Right 존만
/// 이동 거리가 5배가 되어 커밋 시간창 안에 도달하지 못하는 하네스
/// 아티팩트가 생긴다(`src/sim/eval/protocol.rs`의 `run_eval_shot` 참고).
fn run_shot(settings: &BallShooterSettings) -> ShotResult {
    let robot = defaults::robot().expect("robot");
    let mut world = SimWorld::with_physics(robot.clone(), defaults::PhysicsParams::default());
    world.set_use_ground_truth(true);
    if let Some(rail) = robot.arm.rail {
        *world.robot_mut() = RobotState::new(robot.arm.default_joints.clone(), rail.default_x());
    }
    world.shoot_ball(settings);

    let restitution = defaults::ImpactParams::default().racket_effective_restitution;
    let mut observer = LiveObserver::new(&world);
    let mut out = ShotResult::default();
    let mut was_touching = false;
    // 접촉이 감지되는 스텝엔 이미 Rapier가 충격량을 풀어 라켓이 감속한 뒤라,
    // 한 스텝 전 값을 써야 "때리러 들어온 속도"가 된다(WP1 §5-b).
    let mut prev_vr = Vector3::zeros();
    let mut prev_n = Vector3::zeros();
    let mut prev_ball_v = v3(world.ball_velocity());
    let mut impact_point: Option<pingpong_bot::Point3> = None;

    for _ in 0..MAX_STEPS {
        let pre_vr = racket_point_velocity(&world, v3(world.ball_position()));
        let pre_n = racket_normal(&world);
        let pre_ball_v = v3(world.ball_velocity());

        world.step(DT, None);
        if world.swing_committed() {
            out.committed = true;
        }
        let touching = ball_touches_racket(&world);
        if touching && !was_touching && !out.contact {
            out.contact = true;
            out.vrn = prev_vr.dot(&prev_n);
            out.vr_mag = prev_vr.norm();
            let ball = pingpong_bot::Point3::from(v3(world.ball_position()));
            let v_out_desired = Impact::rally_return(ball, prev_ball_v);
            out.vrn_required =
                Impact::required_racket_velocity(prev_ball_v, v_out_desired, prev_n, restitution)
                    .map(|v| v.dot(&prev_n))
                    .unwrap_or(f64::NAN);
            out.v_out_desired = v_out_desired.norm();
            out.desired_clears_net = Impact::clears_net(ball, v_out_desired);
            impact_point = Some(ball);
        }
        // 접촉이 끊긴 첫 스텝의 공 속도 = 실측 리턴 속도.
        if was_touching && !touching && out.v_out_actual == 0.0 {
            let v = v3(world.ball_velocity());
            out.v_out_actual = v.norm();
            if let Some(ball) = impact_point {
                out.actual_clears_net = Impact::clears_net(ball, v);
                if v.norm() > 1e-9 {
                    let rescaled = v * (out.v_out_desired / v.norm());
                    out.rescaled_clears_net = Impact::clears_net(ball, rescaled);
                }
            }
        }
        was_touching = touching;
        prev_vr = pre_vr;
        prev_n = pre_n;
        prev_ball_v = pre_ball_v;
        if observer.observe(&world) {
            break;
        }
    }
    out.points = observer.points();
    out.bounced_own_half = observer.flags.bounced_own_half;
    out.double_hit = observer.flags.double_hit;
    out.cleared_net = observer.flags.cleared_net;
    out.returned_in = observer.flags.returned_in;
    return out;
}

#[derive(Debug, Clone, Copy, Default)]
struct Stats {
    shots: usize,
    committed: usize,
    contact: usize,
    points: u32,
    bounced_own_half: usize,
    double_hit: usize,
    cleared_net: usize,
    returned_in: usize,
    vrn_sum: f64,
    vr_mag_sum: f64,
    ratio_sum: f64,
    ratio_n: usize,
    desired_clears_net: usize,
    actual_clears_net: usize,
    rescaled_clears_net: usize,
    v_out_actual_sum: f64,
    v_out_ratio_sum: f64,
    v_out_n: usize,
}

impl Stats {
    fn push(&mut self, r: ShotResult) {
        self.shots += 1;
        self.committed += usize::from(r.committed);
        self.contact += usize::from(r.contact);
        self.points += u32::from(r.points);
        if r.contact {
            self.bounced_own_half += usize::from(r.bounced_own_half);
            self.double_hit += usize::from(r.double_hit);
            self.cleared_net += usize::from(r.cleared_net);
            self.returned_in += usize::from(r.returned_in);
            self.vrn_sum += r.vrn;
            self.vr_mag_sum += r.vr_mag;
            self.desired_clears_net += usize::from(r.desired_clears_net);
            self.actual_clears_net += usize::from(r.actual_clears_net);
            self.rescaled_clears_net += usize::from(r.rescaled_clears_net);
            if r.vrn_required.is_finite() && r.vrn_required.abs() > 1e-9 {
                self.ratio_sum += r.vrn / r.vrn_required;
                self.ratio_n += 1;
            }
            if r.v_out_desired > 1e-9 && r.v_out_actual > 0.0 {
                self.v_out_actual_sum += r.v_out_actual;
                self.v_out_ratio_sum += r.v_out_actual / r.v_out_desired;
                self.v_out_n += 1;
            }
        }
    }
    fn pct(count: usize, total: usize) -> f64 {
        return 100.0 * count as f64 / total.max(1) as f64;
    }
    fn row(&self, label: &str) -> String {
        return format!(
            "| {label} | {} | {:.1} | {:.1} | {} | {:.1} | {:.1} | {:.1} | {:.1} | {:.3} | {:.3} | {:.3} | {:.3} | {:.3} | {:.1} | {:.1} | {:.1} |",
            self.shots,
            Self::pct(self.committed, self.shots),
            Self::pct(self.contact, self.shots),
            self.points,
            Self::pct(self.bounced_own_half, self.contact),
            Self::pct(self.double_hit, self.contact),
            Self::pct(self.cleared_net, self.contact),
            Self::pct(self.returned_in, self.contact),
            self.vrn_sum / self.contact.max(1) as f64,
            self.vr_mag_sum / self.contact.max(1) as f64,
            self.ratio_sum / self.ratio_n.max(1) as f64,
            self.v_out_actual_sum / self.v_out_n.max(1) as f64,
            self.v_out_ratio_sum / self.v_out_n.max(1) as f64,
            Self::pct(self.desired_clears_net, self.contact),
            Self::pct(self.actual_clears_net, self.contact),
            Self::pct(self.rescaled_clears_net, self.contact),
        );
    }
}

fn header() -> String {
    return "| grid | shots | commit% | contact% | score | own_half%* | dbl_hit%* | \
            cleared%* | in%* | v_r·n | \\|v_r\\| | v_r·n / req | \\|v_out\\| | \
            \\|v_out\\| / desired | desired가 net통과%* | actual이 net통과%* | \
            actual을 desired크기로%* |\n\
            |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"
        .to_string();
}

/// eval 그리드 지터 시드 — WP1과 같은 값을 써서 두 실험이 **같은 30발**을 본다.
const EVAL_SEED: u64 = 0x5741_5031; // "WAP1"

/// eval 30샷 그리드. 지터를 켠다 — `settings_for_zone_shot`은 `index_in_zone`을
/// 버려서 지터 없이는 존당 10발이 전부 동일해 실질 표본이 3발뿐이다(WP1 §5-a).
fn eval_grid() -> (Stats, [Stats; 3]) {
    use rand::SeedableRng;
    let launch = EvalLaunchParams::default();
    let mut rng = rand::rngs::StdRng::seed_from_u64(EVAL_SEED);
    let mut all = Stats::default();
    let mut by_zone = [Stats::default(); 3];
    for (zone, index_in_zone) in Protocol::shot_schedule(EvalMode::Alternating) {
        let settings =
            Protocol::settings_for_zone_shot_jittered(&launch, zone, index_in_zone, &mut rng);
        let result = run_shot(&settings);
        all.push(result);
        by_zone[zone_index(zone)].push(result);
    }
    return (all, by_zone);
}

/// `Zone::zone_index`(crate-private)의 재현.
fn zone_index(zone: Zone) -> usize {
    return match zone {
        Zone::Right => 0,
        Zone::Center => 1,
        Zone::Left => 2,
    };
}

/// 랜덤 슈터 5×5 그리드 — WP1과 동일한 lateral×yaw 격자.
fn random_grid() -> Stats {
    let mut stats = Stats::default();
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
            stats.push(run_shot(&settings));
        }
    }
    return stats;
}

/// WP2b A/B 계측 — 코드 변경 전(거리 랭킹)/후(복합 랭킹)로 두 번 돌린다.
#[test]
#[ignore = "진단 전용 — 55샷 시뮬"]
fn diag_wp2b_ab_metrics() {
    let (eval_all, by_zone) = eval_grid();
    let rnd = random_grid();

    println!("\n### WP2b 랭킹 A/B 계측\n");
    println!("{}", header());
    println!("{}", eval_all.row("eval 30 (all)"));
    for zone in [Zone::Left, Zone::Center, Zone::Right] {
        println!(
            "{}",
            by_zone[zone_index(zone)].row(&format!("eval {}", zone.label()))
        );
    }
    println!("{}", rnd.row("random 5x5"));
    println!(
        "\n`*` 표시 열은 **접촉한 샷** 기준 비율이다 (미접촉 샷은 분모에서 제외).\n\
         `v_r·n / required` 는 1.0이면 rally 리턴에 필요한 법선속도를 전부 달성."
    );
}
