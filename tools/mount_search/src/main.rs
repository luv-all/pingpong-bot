//! mount-search: 리니어 레일 마운트 위치(테이블과의 거리, 높이) 스윕.
//!
//! 4-DOF 팔이 짧은 링크(≈45cm reach) + 실기 스펙 기반 관절속도 한계
//! (~2.88 rad/s) 조합에서 일반적인 랠리 리턴 속도(~2 m/s)조차 특정 자세에서
//! 관절속도 조작성이 나빠 버거워지는 문제가 있었다(2026-07-23 조사).
//! `motion::swing_feasibility`(다중 IK 시드 중 최선 조작성 선택, quintic
//! 없이 "낼 수 있는가"만 봄)를 여러 마운트 후보(레일 높이·테이블과의 거리)에
//! 대해 대표 랠리 시나리오 배터리로 채점해, 어떤 마운트 위치가 가장 넓은
//! 방향/속도 범위를 실기 관절속도 한계 안에서 커버하는지 찾는다.
//!
//! 실기 마운트: 테이블 면보다 약 3cm 위(2026-07-23 실측 보고) — 기본
//! 스윕 범위는 이 값 근방을 포함한다. `defaults::primitive_4dof_with_mount`만
//! 파라미터화돼 있어 `--robot` 선택지는 없다(경진용 primitive 전용).
//!
//! 사용법: cargo run -p mount-search --release
//!         cargo run -p mount-search --release -- --json

mod args;
mod mount_result;
mod scenario;

use anyhow::Result;
use clap::Parser;
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::estimator::Prediction;
use pingpong_bot::robot;
use pingpong_bot::robot::motion;

use args::Args;
use mount_result::MountResult;
use scenario::{Scenario, build_scenarios};

/// 실현 가능(NearSingularity 임계값과 별개, 실기 관절속도 한계 자체) 판정 기준.
/// `motion::physics::NEAR_SINGULARITY_SPEED_RATIO`(2.5)와는 다른 목적 —
/// 여기서는 "특이점 근접"이 아니라 "실제로 안전하게 실행 가능한가"를 좀 더
/// 보수적으로 본다(피크가 한계에 딱 걸치면 토크 여유가 없어 불안정할 수
/// 있음, `plan_swing`의 `fit_end_velocity` 안전계수 0.95와 같은 취지).
const FEASIBLE_RATIO_THRESHOLD: f64 = 1.0;

fn linspace(min: f64, max: f64, steps: usize) -> Vec<f64> {
    if steps <= 1 {
        return vec![min];
    }
    return (0..steps)
        .map(|i| min + (max - min) * i as f64 / (steps - 1) as f64)
        .collect();
}

fn evaluate_mount(
    base_y: f64,
    height_offset_m: f64,
    scenarios: &[Scenario],
) -> Option<MountResult> {
    // `height_offset_m`은 테이블 면 기준 오프셋, 빌더는 월드 z를 받는다.
    let robot =
        defaults::primitive_4dof_with_mount(base_y, table::SURFACE_Z + height_offset_m).ok()?;
    let arm = robot.arm;
    let start = arm.initial_state();
    let start_pose = robot::Pose::new(start.rail_x(), start.joints().clone());

    let ratios: Vec<f64> = scenarios
        .iter()
        .map(|scenario| {
            let prediction = Prediction {
                // IK/속도 조작성 평가에는 임팩트까지 남은 시간이 영향을 주지
                // 않으므로(quintic 궤적 생성 없이 순간 조작성만 봄) 대표값으로
                // 고정한다.
                time_to_impact_secs: 0.2,
                impact_position: scenario.impact,
                incoming_velocity: scenario.incoming_velocity,
            };
            motion::Planner::feasibility(&arm, &prediction, &start_pose)
                .map(|f| f.peak_joint_speed_ratio)
                .unwrap_or(f64::INFINITY)
        })
        .collect();

    let total = ratios.len();
    let feasible_count = ratios
        .iter()
        .filter(|&&r| r <= FEASIBLE_RATIO_THRESHOLD)
        .count();
    let finite: Vec<f64> = ratios.iter().copied().filter(|r| r.is_finite()).collect();
    let mean_peak_ratio = if finite.is_empty() {
        f64::INFINITY
    } else {
        finite.iter().sum::<f64>() / finite.len() as f64
    };
    let worst_peak_ratio = ratios.iter().copied().fold(0.0_f64, f64::max);

    return Some(MountResult {
        base_y,
        height_offset_m,
        feasible_count,
        total,
        mean_peak_ratio,
        worst_peak_ratio,
    });
}

fn main() -> Result<()> {
    let args = Args::parse();
    let scenarios = build_scenarios();

    let heights = linspace(args.height_min, args.height_max, args.height_steps);
    let mut results: Vec<MountResult> =
        linspace(args.base_y_min, args.base_y_max, args.base_y_steps)
            .into_iter()
            .flat_map(|base_y| heights.iter().copied().map(move |h| (base_y, h)))
            .filter_map(|(base_y, height_offset_m)| {
                evaluate_mount(base_y, height_offset_m, &scenarios)
            })
            .collect();

    results.sort_by(|a, b| {
        b.feasible_count.cmp(&a.feasible_count).then_with(|| {
            a.mean_peak_ratio
                .partial_cmp(&b.mean_peak_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    });

    if args.json {
        println!(
            "{}",
            serde_json::to_string_pretty(&results[..results.len().min(args.top_n)])?
        );
    } else {
        println!(
            "mount-search — {} 마운트 후보 x {} 시나리오 스윕 (실현가능 기준: peak_joint_speed_ratio <= {FEASIBLE_RATIO_THRESHOLD})",
            results.len(),
            scenarios.len()
        );
        println!(
            "{:>10} {:>14} {:>18} {:>14} {:>14}",
            "base_y[m]", "height_off[m]", "feasible/total", "mean_ratio", "worst_ratio"
        );
        for result in results.iter().take(args.top_n) {
            println!(
                "{:>10.4} {:>14.4} {:>10}/{:<7} {:>14.3} {:>14.3}",
                result.base_y,
                result.height_offset_m,
                result.feasible_count,
                result.total,
                result.mean_peak_ratio,
                result.worst_peak_ratio
            );
        }
        if let Some(best) = results.first() {
            println!(
                "\n최적 후보: base_y={:.4}m, height_offset={:.4}m ({}/{} 시나리오 실기 관절속도 한계 안에서 실행 가능)",
                best.base_y, best.height_offset_m, best.feasible_count, best.total
            );
        }
    }

    return Ok(());
}
