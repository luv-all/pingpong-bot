//! RailQueue 실기 검증 벤치 — 1차 이동(중앙→최좌단)이 끝나기 전에 2차 명령을
//! 넣어도 RailQueue가 "이전 이동을 항상 끝까지 마친 뒤에만 다음 명령을 보낸다"를
//! 지키는지 확인한다.
//!
//! `calib_rail`과 같은 패턴으로 `AxlRail::open`을 직접 쓴다 — `RealHardware`를
//! 거치지 않아 Dynamixel 팔 정렬 검사와 무관하게 레일만 단독 검증할 수 있다.
//! 설계 문서: docs/superpowers/specs/2026-08-13-rail-command-queue-design.md

mod args;

use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_bot::defaults;
use pingpong_bot::hardware::rail::{AxlRail, RailCalibration, RailConfig, RailQueue};
use pingpong_bot::telemetry::init_tracing;
use tracing::{error, info, warn};

use args::{Args, Scenario};

fn main() -> Result<()> {
    let args = Args::parse();
    init_tracing(args.debug, &["rail_queue_bench", "pingpong_bot"], false);

    let mut rail_cfg = RailConfig::default();
    if let Some(dll_path) = &args.dll_path {
        rail_cfg.dll_path = dll_path.clone();
    }
    let calibration_path = defaults::rail::rail_calibration_path();
    if let Some(calibration) = RailCalibration::load(&calibration_path) {
        info!(
            path = %calibration_path.display(),
            board_zero_domain_m = calibration.board_zero_domain_m,
            "레일 캘리브레이션 파일 적용"
        );
        calibration.apply_to(&mut rail_cfg);
    }

    if !(0.0..1.0).contains(&args.target2_fraction) {
        warn!(
            target2_fraction = args.target2_fraction,
            "target2_fraction은 0.0..1.0 권장 — 최좌단/중앙 밖 값은 안전 클램프로 잘립니다"
        );
    }

    let leftmost_m = rail_cfg.x_min_m;
    let center_m = defaults::rail::RAIL_READY_X_M.clamp(rail_cfg.x_min_m, rail_cfg.x_max_m);
    let target2_m = leftmost_m + (center_m - leftmost_m) * args.target2_fraction;

    info!(
        leftmost_m,
        center_m, target2_m, "벤치 좌표 — 항상 x_min_m..x_max_m 안전 범위 안"
    );

    let mut all_passed = true;
    for scenario in scenarios_to_run(args.scenario) {
        let passed = run_scenario(&rail_cfg, &args, scenario, leftmost_m, center_m, target2_m)?;
        all_passed = all_passed && passed;
    }

    if !all_passed {
        anyhow::bail!("하나 이상의 시나리오가 실패했습니다 — 위 로그에서 FAIL 확인");
    }
    info!("모든 시나리오 통과 — RailQueue가 실기에서 wait-then-send를 지킵니다");
    return Ok(());
}

fn scenarios_to_run(scenario: Scenario) -> Vec<Scenario> {
    return match scenario {
        Scenario::Both => vec![Scenario::Early, Scenario::Late],
        other => vec![other],
    };
}

/// 시나리오 1회 실행. 통과하면 `true`.
fn run_scenario(
    rail_cfg: &RailConfig,
    args: &Args,
    scenario: Scenario,
    leftmost_m: f64,
    center_m: f64,
    target2_m: f64,
) -> Result<bool> {
    let fraction = match scenario {
        Scenario::Early => args.early_fraction,
        Scenario::Late => args.late_fraction,
        Scenario::Both => unreachable!("Both는 scenarios_to_run에서 Early/Late로 분해됨"),
    };
    let preempt_delay_secs = args.duration1_secs * fraction;
    info!(
        ?scenario,
        preempt_delay_secs, "=== 시나리오 시작 — 준비: 중앙으로 블로킹 이동 ==="
    );

    let mut rail = AxlRail::open(rail_cfg.clone()).context("레일 초기화 실패")?;
    // move_abs_m은 기본 최고속(7.5 m/s)로 블로킹 이동해 시나리오 사이에 레일이
    // 갑자기 전속력으로 튄다 — 검증 이동과 같은 duration 기반 속도로 준비한다.
    rail.command_abs_in_secs(center_m, args.prep_duration_secs)
        .context("준비 이동 시작: 중앙")?;
    rail.wait_idle().context("준비 이동 대기: 중앙")?;
    let measured_start_m = rail.read_x_m().context("준비 위치 실측")?;
    info!(measured_start_m, "중앙 준비 완료 — RailQueue로 전환");

    let queue = RailQueue::spawn(rail);
    let scenario_started = Instant::now();

    queue.enqueue(leftmost_m, args.duration1_secs);
    info!(target_m = leftmost_m, duration_secs = args.duration1_secs, "1차 명령 enqueue");

    thread::sleep(Duration::from_secs_f64(preempt_delay_secs));
    queue.enqueue(target2_m, args.duration2_secs);
    info!(
        target_m = target2_m,
        duration_secs = args.duration2_secs,
        elapsed_since_scenario_start_secs = scenario_started.elapsed().as_secs_f64(),
        "2차 명령 enqueue — 1차 이동이 진행 중일 수 있음"
    );

    queue.wait_idle();
    let total_elapsed_secs = scenario_started.elapsed().as_secs_f64();
    let error = queue.take_error();
    drop(queue);

    return match error {
        Some(error) => {
            error!(?scenario, %error, total_elapsed_secs, "FAIL — RailQueue가 에러를 기록했습니다");
            Ok(false)
        }
        None => {
            info!(
                ?scenario,
                total_elapsed_secs,
                "PASS — 1차 이동이 끝난 뒤에만 2차 명령이 실행됐습니다(에러 없음)"
            );
            Ok(true)
        }
    };
}
