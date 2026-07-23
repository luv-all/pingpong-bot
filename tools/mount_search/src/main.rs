//! mount-search: 리니어 레일 마운트 위치(테이블과의 거리, 높이) 스윕.
//!
//! 4-DOF 팔이 짧은 링크(≈45cm reach) + 실기 스펙 기반 관절속도 한계
//! (~2.88 rad/s) 조합에서 일반적인 랠리 리턴 속도(~2 m/s)조차 특정 자세에서
//! 관절속도 조작성이 나빠 버거워지는 문제가 있었다(2026-07-23 조사).
//! `planner::swing_feasibility`(다중 IK 시드 중 최선 조작성 선택, quintic
//! 없이 "낼 수 있는가"만 봄)를 여러 마운트 후보(레일 높이·테이블과의 거리)에
//! 대해 대표 랠리 시나리오 배터리로 채점해, 어떤 마운트 위치가 가장 넓은
//! 방향/속도 범위를 실기 관절속도 한계 안에서 커버하는지 찾는다.
//!
//! 실기 마운트: 테이블 면보다 약 3cm 위(2026-07-23 실측 보고) — 기본
//! 스윕 범위는 이 값 근방을 포함한다. `Arm::competition_with_mount`만
//! 파라미터화돼 있어 `--robot` 선택지는 없다(경진용 primitive 전용).
//!
//! 사용법: cargo run -p mount-search --release
//!         cargo run -p mount-search --release -- --json

use anyhow::Result;
use clap::Parser;
use nalgebra::Vector3;
use pingpong_bot::constants::table;
use pingpong_bot::{Arm, Point3, Prediction, RobotPose, swing_feasibility};
use serde::Serialize;

/// 실현 가능(NearSingularity 임계값과 별개, 실기 관절속도 한계 자체) 판정 기준.
/// `planner::physics::NEAR_SINGULARITY_SPEED_RATIO`(2.5)와는 다른 목적 —
/// 여기서는 "특이점 근접"이 아니라 "실제로 안전하게 실행 가능한가"를 좀 더
/// 보수적으로 본다(피크가 한계에 딱 걸치면 토크 여유가 없어 불안정할 수
/// 있음, `plan_swing`의 `fit_end_velocity` 안전계수 0.95와 같은 취지).
const FEASIBLE_RATIO_THRESHOLD: f64 = 1.0;

#[derive(Parser, Debug)]
#[command(about = "레일 마운트 위치(테이블과의 거리·높이) 스윕으로 최적 위치를 찾는다")]
struct Args {
    /// 테이블과의 거리(y) 후보 최소값 [m] - `BASE_Y` 관례 좌표계.
    #[arg(long, allow_hyphen_values = true, default_value_t = -0.05)]
    base_y_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.10)]
    base_y_max: f64,
    #[arg(long, default_value_t = 7)]
    base_y_steps: usize,

    /// 테이블 면 대비 높이 오프셋 후보 [m] - 실기는 약 +0.03.
    #[arg(long, allow_hyphen_values = true, default_value_t = -0.02)]
    height_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.08)]
    height_max: f64,
    #[arg(long, default_value_t = 6)]
    height_steps: usize,

    #[arg(long)]
    json: bool,

    /// 상위 몇 개 후보를 출력할지.
    #[arg(long, default_value_t = 5)]
    top_n: usize,
}

struct Scenario {
    impact: Point3,
    incoming_velocity: Vector3<f64>,
}

/// 대표 랠리 시나리오 배터리 - 테이블 폭 전역 × 입사 높이(스킷/아크) ×
/// 속도 × 하강각. 특정 슈터 조준 기하(사설 API)에 의존하지 않고, 이
/// 팔이 실제로 마주칠 임팩트 위치/속도 범위를 직접 정의한다.
///
/// 속도·높이 범위는 2026-07-23 실측(사람이 실제로 치는 랠리 속도 연구 +
/// `swing_feasibility` 스윕)에 맞춰 갱신 - 이전 [4.0,5.5] m/s는 실제 사람
/// 랠리 속도(레크리에이션 12-14 m/s)보다 훨씬 느려 비현실적이었다. 높이도
/// "임팩트 높이가 핵심 요인" 발견에 맞춰 정상 범위(테이블 위 10~30cm)로
/// 좁혔다 - 5cm(스킷)·40cm(높은 로브)는 실측상 어떤 마운트로도 항상
/// 불가능했다.
fn build_scenarios() -> Vec<Scenario> {
    let mut scenarios = Vec::new();
    for &x_frac in &[0.15, 0.35, 0.5, 0.65, 0.85] {
        let impact_x = table::WIDTH_X * x_frac;
        for &z_offset in &[0.10, 0.15, 0.20, 0.25, 0.30] {
            let impact_z = table::SURFACE_Z + z_offset;
            for &speed in &[7.0, 8.5, 10.0] {
                for &descend_frac in &[0.10, 0.30] {
                    let incoming_velocity =
                        Vector3::new(0.0, -speed, -speed * descend_frac);
                    scenarios.push(Scenario {
                        impact: Point3::new(impact_x, table::DEFAULT_HIT_PLANE_Y, impact_z),
                        incoming_velocity,
                    });
                }
            }
        }
    }
    return scenarios;
}

#[derive(Debug, Serialize)]
struct MountResult {
    base_y: f64,
    height_offset_m: f64,
    feasible_count: usize,
    total: usize,
    mean_peak_ratio: f64,
    worst_peak_ratio: f64,
}

fn linspace(min: f64, max: f64, steps: usize) -> Vec<f64> {
    if steps <= 1 {
        return vec![min];
    }
    return (0..steps)
        .map(|i| min + (max - min) * i as f64 / (steps - 1) as f64)
        .collect();
}

fn evaluate_mount(base_y: f64, height_offset_m: f64, scenarios: &[Scenario]) -> Option<MountResult> {
    let arm = Arm::competition_with_mount(base_y, height_offset_m).ok()?;
    let start = arm.initial_state();
    let start_pose = RobotPose::new(start.rail_x(), start.joints().clone());

    let mut ratios = Vec::with_capacity(scenarios.len());
    for scenario in scenarios {
        let prediction = Prediction {
            // IK/속도 조작성 평가에는 임팩트까지 남은 시간이 영향을 주지
            // 않으므로(quintic 궤적 생성 없이 순간 조작성만 봄) 대표값으로
            // 고정한다.
            time_to_impact_secs: 0.2,
            impact_position: scenario.impact,
            incoming_velocity: scenario.incoming_velocity,
        };
        let ratio = swing_feasibility(&arm, &prediction, &start_pose)
            .map(|f| f.peak_joint_speed_ratio)
            .unwrap_or(f64::INFINITY);
        ratios.push(ratio);
    }

    let total = ratios.len();
    let feasible_count = ratios.iter().filter(|r| **r <= FEASIBLE_RATIO_THRESHOLD).count();
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

    let mut results: Vec<MountResult> = Vec::new();
    for base_y in linspace(args.base_y_min, args.base_y_max, args.base_y_steps) {
        for height_offset_m in linspace(args.height_min, args.height_max, args.height_steps) {
            if let Some(result) = evaluate_mount(base_y, height_offset_m, &scenarios) {
                results.push(result);
            }
        }
    }

    results.sort_by(|a, b| {
        b.feasible_count
            .cmp(&a.feasible_count)
            .then_with(|| a.mean_peak_ratio.partial_cmp(&b.mean_peak_ratio).unwrap_or(std::cmp::Ordering::Equal))
    });

    if args.json {
        println!("{}", serde_json::to_string_pretty(&results[..results.len().min(args.top_n)])?);
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
