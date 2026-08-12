//! 레일 온디맨드 홈잉 — 재조립 후 등 필요할 때만 수동 실행한다.
//!
//! 물리적 엔드스톱까지 저속 이동해 AXL 알람으로 도달을 감지하고, 그 지점 기준으로
//! `board_zero_domain_m`을 다시 계산해 `data/rail_calibration.json`에 저장한다. 다음
//! 실기 실행부터는 재빌드 없이 이 값이 하드코딩 기본값을 덮어쓴다
//! (`pingpong_bot::defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M`).

mod args;

use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_bot::defaults;
use pingpong_bot::hardware::RealHardware;
use pingpong_bot::hardware::dynamixel::DynamixelConfig;
use pingpong_bot::hardware::rail::{RailCalibration, RailConfig};
use pingpong_bot::telemetry::init_tracing;
use tracing::info;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();
    init_tracing(args.debug, &["calib_rail", "pingpong_bot"], false);

    let mut dxl = DynamixelConfig::default();
    if let Some(port) = &args.port {
        dxl.port = port.clone();
    }
    let mut rail_cfg = RailConfig::default();
    if let Some(dll_path) = &args.dll_path {
        rail_cfg.dll_path = dll_path.clone();
    }

    let end: pingpong_bot::hardware::rail::RailEnd = args.end.into();
    info!(
        port = %dxl.port,
        dll_path = %rail_cfg.dll_path.display(),
        end = ?end,
        "레일 홈잉 시작 — 물리적 엔드스톱까지 저속 이동합니다"
    );

    let mut hardware =
        RealHardware::new(dxl, Some(rail_cfg)).context("하드웨어 초기화 실패")?;
    let result = hardware.home_rail(end).context("레일 홈잉")?;

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
