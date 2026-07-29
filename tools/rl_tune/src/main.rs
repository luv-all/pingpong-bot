//! 안전한 저차원 residual을 학습하는 headless episodic policy search.
//!
//! 기존 플래너가 접촉 위치, IK, 관절/토크/충돌 한계를 책임지고 이 도구는
//! 목표 착지점 x/y와 착지 시간만 CEM으로 탐색한다. 한 에피소드는 공 한 발.

use std::cmp::Ordering;
use std::path::PathBuf;

use anyhow::{Context, Result, ensure};
use clap::Parser;
use pingpong_bot::constants::{BALL_RADIUS, table};
use pingpong_bot::sim::{BallShooterSettings, SimWorld};
use pingpong_bot::{Robot, SwingResidual, defaults};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use rapier3d::prelude::{ColliderHandle, RigidBodyHandle};
use serde::Serialize;

const DT: f64 = 1.0 / 1000.0;
const MAX_STEPS: usize = 3_500;

#[derive(Parser, Debug)]
#[command(about = "Rapier 한 발 보상으로 리턴 residual 정책을 CEM 학습한다")]
struct Args {
    /// CEM 세대 수.
    #[arg(long, default_value_t = 8)]
    generations: usize,
    /// 세대당 정책 후보 수.
    #[arg(long, default_value_t = 16)]
    population: usize,
    /// 다음 분포를 만드는 상위 후보 수.
    #[arg(long, default_value_t = 4)]
    elite: usize,
    /// 후보 하나를 평가할 고정 랜덤 샷 수.
    #[arg(long, default_value_t = 4)]
    shots: usize,
    /// 재현 가능한 샷/정책 샘플 시드.
    #[arg(long, default_value_t = 20260728)]
    seed: u64,
    /// 세대마다 갱신할 정책 JSON.
    #[arg(long, default_value = "rl_policy.json")]
    output: PathBuf,
}

#[derive(Debug, Clone, Copy, Default)]
struct ShotOutcome {
    incoming_valid: bool,
    committed: bool,
    contact: bool,
    returned: bool,
    cleared_net: bool,
    returned_in: bool,
    bounced_own_half: bool,
    bounce_xy: Option<[f64; 2]>,
    peak_outgoing_y_mps: f64,
}

#[derive(Debug, Clone)]
struct Candidate {
    residual: SwingResidual,
    mean_reward: f64,
    successes: usize,
}

#[derive(Debug, Serialize)]
struct SerializableResidual {
    bounce_x_offset_m: f64,
    bounce_y_offset_m: f64,
    bounce_time_scale: f64,
}

#[derive(Debug, Serialize)]
struct PolicyFile {
    algorithm: &'static str,
    generation: usize,
    seed: u64,
    mean_reward: f64,
    successes: usize,
    evaluation_shots: usize,
    residual: SerializableResidual,
}

impl From<SwingResidual> for SerializableResidual {
    fn from(value: SwingResidual) -> Self {
        return Self {
            bounce_x_offset_m: value.bounce_x_offset_m,
            bounce_y_offset_m: value.bounce_y_offset_m,
            bounce_time_scale: value.bounce_time_scale,
        };
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    ensure!(args.generations > 0, "--generations는 1 이상");
    ensure!(args.population >= 2, "--population은 2 이상");
    ensure!(args.elite > 0 && args.elite <= args.population, "--elite 범위");
    ensure!(args.shots > 0, "--shots는 1 이상");

    let robot = defaults::primitive_4dof().context("4-dof 로봇 생성")?;
    let mut shot_rng = StdRng::seed_from_u64(args.seed);
    let base = BallShooterSettings::default();
    // 같은 세대의 모든 후보가 같은 공을 받아야 보상 차이가 정책 때문임을
    // 보장할 수 있다. 배터리는 학습 전체에서 고정한다(common random numbers).
    let battery: Vec<_> = (0..args.shots)
        .map(|_| base.randomized(&mut shot_rng))
        .collect();

    let mut policy_rng = StdRng::seed_from_u64(args.seed ^ 0xC0FF_EE12_3456_7890);
    // [bounce x offset, bounce y offset, bounce time scale]
    let mut mean = [0.0, 0.0, 1.0];
    let mut std = [0.18, 0.25, 0.20];
    let mut best_ever: Option<Candidate> = None;

    for generation in 0..args.generations {
        let mut residuals = Vec::with_capacity(args.population);
        // 현재 평균과 기존 플래너를 매 세대 다시 넣어 우연한 샘플 때문에
        // 이미 찾은 안전한 영역을 완전히 잃지 않게 한다.
        residuals.push(from_vector(mean));
        if args.population > 1 {
            residuals.push(SwingResidual::default());
        }
        while residuals.len() < args.population {
            residuals.push(from_vector([
                mean[0] + std[0] * standard_normal(&mut policy_rng),
                mean[1] + std[1] * standard_normal(&mut policy_rng),
                mean[2] + std[2] * standard_normal(&mut policy_rng),
            ]));
        }

        let mut candidates = evaluate_population(&robot, &battery, &residuals);
        candidates.sort_by(|left, right| {
            right
                .mean_reward
                .partial_cmp(&left.mean_reward)
                .unwrap_or(Ordering::Equal)
                .then_with(|| right.successes.cmp(&left.successes))
        });
        let best = candidates[0].clone();
        if best_ever
            .as_ref()
            .is_none_or(|old| best.mean_reward > old.mean_reward)
        {
            best_ever = Some(best.clone());
        }

        let elite = &candidates[..args.elite];
        let elite_vectors: Vec<_> = elite.iter().map(|item| to_vector(item.residual)).collect();
        let mut next_mean = [0.0; 3];
        for values in &elite_vectors {
            for axis in 0..3 {
                next_mean[axis] += values[axis] / args.elite as f64;
            }
        }
        let mut next_std = [0.0; 3];
        for values in &elite_vectors {
            for axis in 0..3 {
                next_std[axis] +=
                    (values[axis] - next_mean[axis]).powi(2) / args.elite as f64;
            }
        }
        for axis in 0..3 {
            next_std[axis] = next_std[axis].sqrt();
            // 분포를 부드럽게 갱신하고 탐색이 너무 빨리 죽지 않게 최소 폭 유지.
            mean[axis] = 0.25 * mean[axis] + 0.75 * next_mean[axis];
            std[axis] = (0.25 * std[axis] + 0.75 * next_std[axis]).max([0.01, 0.01, 0.02][axis]);
        }
        mean = to_vector(from_vector(mean));

        println!(
            "gen {:>2}/{:<2} reward={:>6.2} success={}/{} residual=[x={:+.3}m y={:+.3}m time={:.3}x] std=[{:.3}, {:.3}, {:.3}]",
            generation + 1,
            args.generations,
            best.mean_reward,
            best.successes,
            args.shots,
            best.residual.bounce_x_offset_m,
            best.residual.bounce_y_offset_m,
            best.residual.bounce_time_scale,
            std[0],
            std[1],
            std[2],
        );
        write_policy(&args, generation + 1, best_ever.as_ref().expect("best"))?;
    }

    let best = best_ever.expect("적어도 한 세대");
    println!(
        "\n완료: reward={:.2}, success={}/{}, policy={}",
        best.mean_reward,
        best.successes,
        args.shots,
        args.output.display()
    );
    return Ok(());
}

fn evaluate_population(
    robot: &Robot,
    battery: &[BallShooterSettings],
    residuals: &[SwingResidual],
) -> Vec<Candidate> {
    let threads = std::thread::available_parallelism().map_or(1, |count| count.get());
    let chunk_size = residuals.len().div_ceil(threads).max(1);
    return std::thread::scope(|scope| {
        let handles: Vec<_> = residuals
            .chunks(chunk_size)
            .map(|chunk| {
                scope.spawn(move || {
                    chunk
                        .iter()
                        .copied()
                        .map(|residual| evaluate_candidate(robot, battery, residual))
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|handle| handle.join().expect("CEM worker"))
            .collect()
    });
}

fn evaluate_candidate(
    robot: &Robot,
    battery: &[BallShooterSettings],
    residual: SwingResidual,
) -> Candidate {
    let target = target_bounce_xy(residual);
    let mut total_reward = 0.0;
    let mut successes = 0;
    for settings in battery {
        let outcome = run_shot(robot, settings, residual);
        total_reward += reward(outcome, target);
        successes += usize::from(
            outcome.incoming_valid && outcome.committed && outcome.returned_in,
        );
    }
    return Candidate {
        residual,
        mean_reward: total_reward / battery.len() as f64,
        successes,
    };
}

fn run_shot(
    robot: &Robot,
    settings: &BallShooterSettings,
    residual: SwingResidual,
) -> ShotOutcome {
    let mut world = SimWorld::new(robot.clone());
    world.set_use_ground_truth(true);
    world.set_swing_residual(residual);

    let ball_collider = collider_for_parent(&world, world.ball_handle);
    let racket_collider = collider_for_parent(&world, world.racket_handle);
    let table_collider = world
        .collider_set
        .iter()
        .find_map(|(handle, collider)| {
            let cuboid = collider.shape().as_cuboid()?;
            ((f64::from(cuboid.half_extents.x) - table::WIDTH_X * 0.5).abs() < 1e-5
                && (f64::from(cuboid.half_extents.y) - table::LENGTH_Y * 0.5).abs() < 1e-5)
                .then_some(handle)
        })
        .expect("table collider");

    world.shoot_ball(settings);
    let mut outcome = ShotOutcome::default();
    let net_y = (table::LENGTH_Y * 0.5) as f32;
    let net_top_z = (table::SURFACE_Z + table::NET_HEIGHT + BALL_RADIUS) as f32;
    let mut previous_y = world.ball_position().y;
    let mut incoming_crossed_net = false;

    for _ in 0..MAX_STEPS {
        world.step(DT, None);
        outcome.committed |= world.swing_committed();
        let position = world.ball_position();
        let velocity = world.ball_velocity();
        let on_table = world
            .narrow_phase
            .contact_pair(ball_collider, table_collider)
            .is_some_and(|pair| pair.has_any_active_contact());

        if !outcome.contact {
            if previous_y > net_y && position.y <= net_y {
                incoming_crossed_net = position.z > net_top_z;
            }
            if incoming_crossed_net && position.y > 0.0 && position.y < net_y && on_table {
                outcome.incoming_valid = true;
            }
        }

        let touching_racket = world
            .narrow_phase
            .contact_pair(ball_collider, racket_collider)
            .is_some_and(|pair| pair.has_any_active_contact());
        if touching_racket {
            outcome.contact = true;
        }
        if outcome.contact && velocity.y > 0.0 {
            outcome.returned = true;
            outcome.peak_outgoing_y_mps =
                outcome.peak_outgoing_y_mps.max(f64::from(velocity.y));
        }
        if outcome.returned && previous_y < net_y && position.y >= net_y {
            outcome.cleared_net = position.z > net_top_z;
        }
        if outcome.contact && on_table && position.y < net_y {
            outcome.bounced_own_half = true;
            break;
        }
        if outcome.cleared_net && on_table && position.y >= net_y {
            outcome.returned_in = f64::from(position.y) < table::LENGTH_Y;
            outcome.bounce_xy = Some([f64::from(position.x), f64::from(position.y)]);
            break;
        }
        previous_y = position.y;
    }
    return outcome;
}

fn reward(outcome: ShotOutcome, target: [f64; 2]) -> f64 {
    let mut value = 0.0;
    value += if outcome.incoming_valid { 1.0 } else { -4.0 };
    value += if outcome.committed { 1.0 } else { -3.0 };
    value += if outcome.contact { 2.0 } else { -4.0 };
    if outcome.returned {
        value += 2.0;
        // 무제한 힘 대신 상대 코트 방향 속도에 최대 2점까지만 준다.
        value += (outcome.peak_outgoing_y_mps / 3.0).clamp(0.0, 2.0);
    }
    if outcome.cleared_net {
        value += 4.0;
    }
    if outcome.returned_in {
        value += 10.0;
    }
    if let Some(actual) = outcome.bounce_xy {
        let error = (actual[0] - target[0]).hypot(actual[1] - target[1]);
        value -= 3.0 * error;
    }
    if outcome.bounced_own_half {
        value -= 8.0;
    }
    return value;
}

fn collider_for_parent(
    world: &SimWorld,
    parent: RigidBodyHandle,
) -> ColliderHandle {
    return world
        .collider_set
        .iter()
        .find_map(|(handle, collider)| (collider.parent() == Some(parent)).then_some(handle))
        .expect("parent collider");
}

fn target_bounce_xy(residual: SwingResidual) -> [f64; 2] {
    let residual = residual.clamped();
    return [
        (table::WIDTH_X * 0.5 + residual.bounce_x_offset_m)
            .clamp(BALL_RADIUS, table::WIDTH_X - BALL_RADIUS),
        (table::LENGTH_Y * 0.75 + residual.bounce_y_offset_m)
            .clamp(table::LENGTH_Y * 0.5 + BALL_RADIUS, table::LENGTH_Y - BALL_RADIUS),
    ];
}

fn standard_normal(rng: &mut impl Rng) -> f64 {
    // Box-Muller. 0은 ln에 들어가지 않도록 열린 쪽을 직접 제한한다.
    let u1 = rng.gen_range(f64::EPSILON..1.0);
    let u2 = rng.gen_range(0.0..1.0);
    return (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos();
}

fn from_vector(values: [f64; 3]) -> SwingResidual {
    return SwingResidual {
        bounce_x_offset_m: values[0],
        bounce_y_offset_m: values[1],
        bounce_time_scale: values[2],
    }
    .clamped();
}

fn to_vector(residual: SwingResidual) -> [f64; 3] {
    let residual = residual.clamped();
    return [
        residual.bounce_x_offset_m,
        residual.bounce_y_offset_m,
        residual.bounce_time_scale,
    ];
}

fn write_policy(args: &Args, generation: usize, candidate: &Candidate) -> Result<()> {
    let policy = PolicyFile {
        algorithm: "cem_residual_policy_search_v1",
        generation,
        seed: args.seed,
        mean_reward: candidate.mean_reward,
        successes: candidate.successes,
        evaluation_shots: args.shots,
        residual: candidate.residual.into(),
    };
    let bytes = serde_json::to_vec_pretty(&policy).context("정책 JSON 직렬화")?;
    std::fs::write(&args.output, bytes)
        .with_context(|| format!("정책 저장: {}", args.output.display()))?;
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reward_orders_success_above_contact_only() {
        let contact = ShotOutcome {
            incoming_valid: true,
            committed: true,
            contact: true,
            ..ShotOutcome::default()
        };
        let success = ShotOutcome {
            returned: true,
            cleared_net: true,
            returned_in: true,
            bounce_xy: Some(target_bounce_xy(SwingResidual::default())),
            peak_outgoing_y_mps: 4.0,
            ..contact
        };
        let target = target_bounce_xy(SwingResidual::default());
        assert!(reward(success, target) > reward(contact, target) + 10.0);
    }

    #[test]
    fn sampled_policy_is_bounded() {
        let residual = from_vector([99.0, -99.0, f64::NAN]);
        assert_eq!(residual.bounce_x_offset_m, 0.45);
        assert_eq!(residual.bounce_y_offset_m, -0.55);
        assert_eq!(residual.bounce_time_scale, 1.0);
    }
}
