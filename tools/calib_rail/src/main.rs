//! 레일 온디맨드 홈잉 — 재조립 후 등 필요할 때만 수동 실행한다.
//!
//! 물리적 엔드스톱까지 저속 이동해 AXL 알람으로 도달을 감지하고, 그 지점 기준으로
//! `board_zero_domain_m`을 다시 계산해 `data/rail_calibration.json`에 저장한다. 다음
//! 실기 실행부터는 재빌드 없이 이 값이 하드코딩 기본값을 덮어쓴다
//! (`pingpong_bot::defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M`).
//!
//! `AxlRail::open`만 쓴다 — Dynamixel 팔은 레일 홈잉과 무관하고,
//! `RealHardware::new`를 거치면 팔의 듀얼 모터 정렬 검사(`verify_mirror_alignment`)
//! 때문에 팔이 정렬 안 됐다는 이유로 레일 홈잉 자체가 막힌다.
//!
//! `--check-alarm`을 주면 홈잉을 하지 않고 AXL 서보 알람 비트만 확인·해제한다 —
//! 정상 이동 명령이 에러 없이 반환되는데도 레일이 실제로 움직이지 않을 때, 원인이
//! AXL 쪽에 남은 래치된 알람인지 확인하는 진단용이다.

mod args;

use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_bot::defaults;
use pingpong_bot::hardware::rail::{AxlRail, RailCalibration, RailConfig, RailEnd};
use pingpong_bot::telemetry::init_tracing;
use tracing::info;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();
    init_tracing(args.debug, &["calib_rail", "pingpong_bot"], false);

    let mut rail_cfg = RailConfig::default();
    if let Some(dll_path) = &args.dll_path {
        rail_cfg.dll_path = dll_path.clone();
    }

    if args.check_alarm {
        return check_alarm(rail_cfg);
    }

    let end: RailEnd = args.end.into();
    home_and_save(rail_cfg, end)
}

/// 홈잉 없이 AXL 서보 알람 비트만 읽고, 켜져 있으면 해제까지 시도한다.
fn check_alarm(rail_cfg: RailConfig) -> Result<()> {
    info!(
        dll_path = %rail_cfg.dll_path.display(),
        "AXL 서보 알람 상태 확인 — 이동 없음"
    );
    let mut rail = AxlRail::open(rail_cfg).context("레일 초기화 실패")?;

    let alarm = rail.alarm_status().context("알람 상태 조회")?;
    info!(alarm, "AXL 서보 알람 상태");
    if !alarm {
        info!("알람 없음 — 레일이 움직이지 않는다면 AXL/소프트웨어 쪽 문제는 아닙니다");
        return Ok(());
    }

    info!("알람이 켜져 있습니다 — 해제를 시도합니다");
    rail.clear_alarm().context("알람 해제")?;
    let alarm_after = rail.alarm_status().context("알람 상태 재조회")?;
    info!(
        alarm_after,
        "알람 해제 시도 후 상태 — false여야 정상입니다"
    );
    return Ok(());
}

fn home_and_save(rail_cfg: RailConfig, end: RailEnd) -> Result<()> {
    info!(
        dll_path = %rail_cfg.dll_path.display(),
        end = ?end,
        "레일 홈잉 시작 — 물리적 엔드스톱까지 저속 이동합니다"
    );

    let mut rail = AxlRail::open(rail_cfg).context("레일 초기화 실패")?;
    let result = rail.home(end).context("레일 홈잉")?;

    let measured_unix_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let calibration = RailCalibration::from_home(
        end,
        result.board_position_m,
        result.board_zero_domain_m,
        measured_unix_secs,
    );
    let path = defaults::rail::rail_calibration_path();
    calibration
        .save(&path)
        .with_context(|| format!("레일 캘리브레이션 저장: {}", path.display()))?;

    info!(
        path = %path.display(),
        board_position_m = result.board_position_m,
        board_zero_domain_m = result.board_zero_domain_m,
        end = ?end,
        "레일 홈잉 완료 — 캘리브레이션 저장"
    );
    return Ok(());
}
