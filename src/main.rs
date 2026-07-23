//! # pingpong-bot 런타임
//!
//! 배선·숫자는 [`pingpong_bot::entry`] SSOT. 머신만 [`local`] / CLI 오버레이.
//!
//! ```bash
//! cargo run -p pingpong-bot
//! cargo run -p pingpong-bot -- --robot 4-dof
//! cargo run -p pingpong-bot -- --mode real --dxl-port COM8
//! ```

mod local;

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use pingpong_bot::{
    Arm, MountPreset, RobotBuilder, SimRuntimeControls, SimSession, SimSessionConfig, UrdfRobot,
    competition_arm, competition_intercept, competition_physics, find_robot,
    install_competition_tunables, new_shutdown_flag, robot_ids_csv,
};
#[cfg(feature = "gui")]
use pingpong_bot::{SimViewerOptions, run_sim_viewer};
#[cfg(feature = "real")]
use pingpong_bot::{Hardware, RealHardware, competition_detector, competition_dynamixel};
use tracing::info;
#[cfg(not(feature = "gui"))]
use tracing::warn;
use tracing_subscriber::EnvFilter;

use local::{DEFAULT_LOCAL_PATH, LocalMachine};

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
    /// 카탈로그 로봇 id (`competition` | `4-dof` | `urdf-test`).
    #[arg(long, default_value = "competition")]
    robot: String,
    /// 머신 로컬 오버레이 (포트 등). 없으면 `config/local.toml`을 시도.
    #[arg(long, value_name = "PATH")]
    local: Option<PathBuf>,
    /// Dynamixel 포트 오버라이드 (`local` / entry 기본보다 우선).
    #[arg(long)]
    dxl_port: Option<String>,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("info".parse()?))
        .init();

    install_competition_tunables();
    let args = Args::parse();
    let local = load_local(&args)?;

    match args.mode {
        ModeArg::Sim => run_sim_entry(&args)?,
        ModeArg::Real => run_real_entry(&args, local.as_ref())?,
    }
    return Ok(());
}

fn load_local(args: &Args) -> Result<Option<LocalMachine>> {
    if let Some(path) = &args.local {
        return Ok(Some(LocalMachine::load(path)?));
    }
    return LocalMachine::load_optional(PathBuf::from(DEFAULT_LOCAL_PATH).as_path());
}

fn load_robot(robot_id: &str) -> Result<(Arc<Arm>, Option<Arc<UrdfRobot>>)> {
    if robot_id == "competition" {
        return Ok((
            Arc::new(competition_arm().context("competition arm")?),
            None,
        ));
    }
    let entry = find_robot(robot_id).with_context(|| {
        format!("알 수 없는 robot id `{robot_id}` (가능: {})", robot_ids_csv())
    })?;
    let path = entry
        .urdf_path(env!("CARGO_MANIFEST_DIR"))
        .context("이 robot id는 URDF 경로가 필요합니다")?;
    let built = RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(entry.ee_link)
        .mount_preset(MountPreset::Rep103AtTableEnd)
        .max_joint_speed(entry.max_joint_speed)
        .build()
        .context("RobotBuilder")?;
    return Ok((built.arm, built.urdf));
}

fn run_sim_entry(args: &Args) -> Result<()> {
    let physics = competition_physics();
    let (arm, urdf) = load_robot(&args.robot)?;
    info!(
        mode = "sim",
        robot = %args.robot,
        restitution = physics.restitution,
        "entry SSOT"
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
        Arc::clone(&arm),
        urdf.clone(),
        Arc::clone(&controls),
        Arc::clone(&shutdown),
        physics,
    );
    {
        let world_arc = session.world();
        let mut world = world_arc.lock().expect("sim 월드");
        world.set_intercept_window(competition_intercept());
        world.set_use_ground_truth(true);
    }
    info!(robot = %args.robot, "sim kiss3d");
    #[cfg(feature = "gui")]
    {
        run_sim_viewer(SimViewerOptions {
            world: session.world(),
            controls,
            shutdown,
            urdf,
        })
        .map_err(anyhow::Error::msg)?;
    }
    #[cfg(not(feature = "gui"))]
    {
        let _ = (session, controls, shutdown, arm, urdf);
        warn!("gui feature 없음 — headless entry sim은 세션만 생성");
    }
    return Ok(());
}

#[cfg(feature = "real")]
fn run_real_entry(args: &Args, local: Option<&LocalMachine>) -> Result<()> {
    let mut dxl = competition_dynamixel();
    if let Some(port) = args
        .dxl_port
        .as_ref()
        .or_else(|| local.and_then(|l| l.dxl_port.as_ref()))
    {
        dxl.port = port.clone();
    }
    info!(port = %dxl.port, "entry real Dynamixel (mirror ID1↔ID2)");
    let mut hardware = RealHardware::new(dxl, None).context("RealHardware")?;
    let pose = hardware.read_pose().context("read pose")?;
    info!(joints = ?pose.joints.values, "pose");
    let _ = competition_detector();
    return Ok(());
}

#[cfg(not(feature = "real"))]
fn run_real_entry(_args: &Args, _local: Option<&LocalMachine>) -> Result<()> {
    anyhow::bail!("real 모드는 `--features real`로 빌드해야 합니다");
}
