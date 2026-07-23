//! # pingpong-bot 런타임
//!
//! 배선·숫자는 [`pingpong_bot::defaults`] SSOT. 포트 등은 CLI로만 덮어쓴다.
//!
//! ```bash
//! cargo run -p pingpong-bot
//! cargo run -p pingpong-bot -- --mode real --dxl-port COM8
//! ```

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use pingpong_bot::{
    SimRuntimeControls, SimSession, SimSessionConfig, intercept, new_shutdown_flag, physics, robot,
};
#[cfg(feature = "gui")]
use pingpong_bot::{SimViewerOptions, run_sim_viewer};
#[cfg(feature = "real")]
use pingpong_bot::{Hardware, RealHardware, detector, dynamixel, rail};
use tracing::info;
#[cfg(not(feature = "gui"))]
use tracing::warn;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Clone, Copy, ValueEnum)]
enum ModeArg {
    Sim,
    Real,
}

/// CLI 인자.
#[derive(Parser)]
#[command(name = "pingpong-bot", about = "협력 랠리 핑퐁 로봇 런타임")]
struct Args {
    /// sim | real
    #[arg(long, value_enum, default_value = "sim")]
    mode: ModeArg,
    /// Dynamixel 포트 오버라이드 (`defaults::dynamixel().port`보다 우선).
    #[arg(long)]
    dxl_port: Option<String>,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("info".parse()?))
        .init();

    let args = Args::parse();

    match args.mode {
        ModeArg::Sim => run_sim_entry()?,
        ModeArg::Real => run_real_entry(&args)?,
    }
    return Ok(());
}

fn run_sim_entry() -> Result<()> {
    let physics = physics();
    let robot = robot().context("defaults::robot")?;
    info!(
        mode = "sim",
        restitution = physics.restitution,
        "defaults SSOT"
    );
    let controls = Arc::new(Mutex::new(SimRuntimeControls::default()));
    let shutdown = new_shutdown_flag();
    let session = SimSession::with_physics(
        SimSessionConfig {
            physics_hz: 1000.0,
            frame_hz: 120.0,
            time_scale: 1.0,
            camera_count: 3,
        },
        robot.clone(),
        Arc::clone(&controls),
        Arc::clone(&shutdown),
        physics,
    );
    {
        let world_arc = session.world();
        let mut world = world_arc.lock().expect("sim 월드");
        world.set_intercept_window(intercept());
        world.set_use_ground_truth(true);
    }
    info!("sim kiss3d");
    #[cfg(feature = "gui")]
    {
        run_sim_viewer(SimViewerOptions {
            world: session.world(),
            controls,
            shutdown,
            urdf: robot.urdf,
        })
        .map_err(anyhow::Error::msg)?;
    }
    #[cfg(not(feature = "gui"))]
    {
        let _ = (session, controls, shutdown, robot);
        warn!("gui feature 없음 — headless sim은 세션만 생성");
    }
    return Ok(());
}

#[cfg(feature = "real")]
fn run_real_entry(args: &Args) -> Result<()> {
    let mut dxl = dynamixel();
    if let Some(port) = &args.dxl_port {
        dxl.port = port.clone();
    }
    info!(port = %dxl.port, "defaults real Dynamixel (mirror ID1↔ID2)");
    let mut hardware = RealHardware::new(dxl, Some(rail())).context("RealHardware")?;
    let pose = hardware.read_pose().context("read pose")?;
    info!(joints = ?pose.joints.values, "pose");
    let _ = detector();
    return Ok(());
}

#[cfg(not(feature = "real"))]
fn run_real_entry(_args: &Args) -> Result<()> {
    anyhow::bail!("real 모드는 `--features real`로 빌드해야 합니다");
}
