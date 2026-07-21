//! AXL 리니어 레일 수동 조그 — 절대/상대 이동으로 배선·한계·클램프 검증.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail, ensure};
use clap::Parser;
use pingpong_bot::hardware::rail::load_rail_config;
use pingpong_bot::hardware::AxlRail;
use tracing_subscriber::EnvFilter;

#[derive(Parser)]
#[command(
    name = "jog_rail",
    about = "AXL 리니어 레일 수동 조그",
    group(
        clap::ArgGroup::new("move")
            .required(true)
            .args(["position_m", "delta_m"])
    )
)]
struct Args {
    /// `[hardware.rail]`이 있는 런타임 TOML.
    #[arg(long, default_value = "config/real-hardware.toml")]
    config: PathBuf,
    /// 목표 X 위치 [m] (소프트 리밋으로 클램프).
    #[arg(long)]
    position_m: Option<f64>,
    /// 현재 위치 기준 상대 이동량 [m].
    #[arg(long)]
    delta_m: Option<f64>,
    /// DLL·Dynamixel 없이 클램프·이동 경로만 검증.
    #[arg(long)]
    dry_run: bool,
}

fn resolve_rail_config(path: &Path) -> Result<pingpong_bot::hardware::rail::RailConfig> {
    let mut cfg = load_rail_config(path)
        .map_err(anyhow::Error::msg)
        .with_context(|| format!("레일 설정 읽기 실패: {}", path.display()))?;
    if cfg.dll_path.is_relative() {
        let parent = path.parent().unwrap_or_else(|| Path::new("."));
        cfg.dll_path = parent.join(&cfg.dll_path);
    }
    return Ok(cfg);
}

fn run(args: Args) -> Result<()> {
    let cfg = resolve_rail_config(&args.config)?;
    if !args.dry_run && !cfg.enabled {
        bail!("enabled=true 필요");
    }

    let mut rail = if args.dry_run {
        AxlRail::dry_run(cfg.clone())
    } else {
        AxlRail::open(cfg.clone())
    }
    .map_err(anyhow::Error::msg)
    .context("AxlRail 초기화 실패")?;

    let before_m = rail
        .read_x_m()
        .map_err(anyhow::Error::msg)
        .context("현재 레일 위치 읽기 실패")?;
    tracing::info!(
        x_m = before_m,
        x_min_m = cfg.x_min_m,
        x_max_m = cfg.x_max_m,
        "이동 전"
    );

    let commanded_m = if let Some(position_m) = args.position_m {
        ensure!(position_m.is_finite(), "position-m는 유한해야 합니다");
        rail.move_abs_m(position_m)
    } else if let Some(delta_m) = args.delta_m {
        ensure!(delta_m.is_finite(), "delta-m는 유한해야 합니다");
        rail.move_rel_m(delta_m)
    } else {
        bail!("--position-m 또는 --delta-m를 지정하세요");
    }
    .map_err(anyhow::Error::msg)
    .context("레일 이동 실패")?;

    let after_m = rail
        .read_x_m()
        .map_err(anyhow::Error::msg)
        .context("최종 레일 위치 읽기 실패")?;
    tracing::info!(
        commanded_m,
        x_m = after_m,
        x_min_m = cfg.x_min_m,
        x_max_m = cfg.x_max_m,
        "이동 후"
    );
    println!("rail_x_m={commanded_m}");
    return Ok(());
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("info".parse()?))
        .init();
    return run(Args::parse());
}
