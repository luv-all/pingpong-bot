//! # pingpong-bot 런타임
//!
//! 배선·숫자는 [`pingpong_bot::defaults`] SSOT. 포트 등은 CLI로만 덮어쓴다.
//!
//! ```bash
//! cargo run -p pingpong-bot
//! cargo run -p pingpong-bot -- --mode real --dxl-port COM8
//! # 샷별 launch/commit/포기 로그 (기본 info). 더 자세히:
//! cargo run -p pingpong-bot -- --debug
//! ```

mod cli;

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use clap::Parser;
#[cfg(feature = "real")]
use pingpong_bot::camera;
#[cfg(feature = "real")]
use pingpong_bot::defaults::detector_for;
use pingpong_bot::defaults::{PhysicsParams, robot};
#[cfg(feature = "real")]
use pingpong_bot::hardware::dynamixel::DynamixelConfig;
#[cfg(feature = "real")]
use pingpong_bot::hardware::rail::RailConfig;
#[cfg(feature = "real")]
use pingpong_bot::hardware::{Hardware, RealHardware};
use pingpong_bot::motion::InterceptWindow;
#[cfg(feature = "gui")]
use pingpong_bot::sim::gui::{SimViewer, SimViewerOptions};
use pingpong_bot::sim::session::{SimRuntimeControls, SimSession, SimSessionConfig};
use pingpong_bot::telemetry::init_tracing;
use tracing::info;
#[cfg(not(feature = "gui"))]
use tracing::warn;

use cli::{Args, ModeArg};

fn main() -> Result<()> {
    let args = Args::parse();
    init_tracing(args.debug, &["pingpong_bot"]);
    if args.debug {
        info!("debug 로그 활성");
    }

    match args.mode {
        ModeArg::Sim => run_sim_entry()?,
        ModeArg::Real => run_real_entry(&args)?,
    }
    return Ok(());
}

fn run_sim_entry() -> Result<()> {
    let physics = PhysicsParams::default();
    let robot = robot().context("defaults::robot")?;
    info!(
        mode = "sim",
        restitution = physics.restitution,
        "defaults SSOT"
    );
    let controls = Arc::new(Mutex::new(SimRuntimeControls::default()));
    let shutdown = SimRuntimeControls::new_shutdown();
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
        world.set_intercept_window(InterceptWindow::default());
        world.set_use_ground_truth(true);
    }
    info!("sim kiss3d");
    #[cfg(feature = "gui")]
    {
        SimViewer::run(SimViewerOptions {
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
    let mut dxl = DynamixelConfig::default();
    if let Some(port) = &args.dxl_port {
        dxl.port = port.clone();
    }
    info!(port = %dxl.port, "defaults real Dynamixel (mirror ID1↔ID2)");
    let arm = robot().context("defaults::robot")?.arm;
    let mut hardware =
        RealHardware::new(dxl, Some(RailConfig::default()), arm).context("RealHardware")?;
    let pose = hardware.read_pose().context("read pose")?;
    info!(joints = ?pose.joints.values, "pose");
    let _ = detector_for(camera::Id(0)).context("detector_for cam0")?;
    return Ok(());
}

#[cfg(not(feature = "real"))]
fn run_real_entry(_args: &Args) -> Result<()> {
    anyhow::bail!("real 모드는 `--features real`로 빌드해야 합니다");
}
