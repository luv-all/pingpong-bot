//! # pingpong-bot 런타임
//!
//! 배선·숫자는 [`pingpong_bot::defaults`] SSOT. 포트 등은 CLI로만 덮어쓴다.
//!
//! ```bash
//! cargo run -p pingpong-bot
//! # 실기 연속 급구 (완주·센터 복귀 후 다음 공). 모터를 안 움직이는 리허설은 --dry-run.
//! cargo run -p pingpong-bot -- --mode real --dxl-port COM8
//! cargo run -p pingpong-bot -- --mode real --dry-run
//! # 샷별 launch/commit/포기 로그 (기본 info). 더 자세히:
//! cargo run -p pingpong-bot -- --debug
//! ```

mod cli;
#[cfg(feature = "real")]
mod real;

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, ensure};
use clap::Parser;
use pingpong_bot::defaults::{PhysicsParams, robot_with_rail_frame};
#[cfg(feature = "gui")]
use pingpong_bot::sim::gui::{SimViewer, SimViewerOptions};
use pingpong_bot::sim::session::{SimRuntimeControls, SimSession, SimSessionConfig};
use pingpong_bot::telemetry::init_tracing;
use tracing::info;
#[cfg(not(feature = "gui"))]
use tracing::warn;

use cli::{Args, ModeArg};

fn main() -> Result<()> {
    // 관전용 sim 창 자식 — kiss3d와 OpenCV highgui가 둘 다 메인 스레드를 요구해서
    // 한 프로세스에 못 띄운다. clap 파싱 전에 가로챈다 (사용자 대상 플래그가 아니다).
    #[cfg(feature = "real")]
    if std::env::args().any(|arg| arg == real::SIM_CHILD_FLAG) {
        init_tracing(false, &["pingpong_bot"]);
        return real::run_sim_child();
    }

    let args = Args::parse();
    validate_setup_args(&args)?;
    init_tracing(args.debug, &["pingpong_bot"]);
    if args.debug {
        info!("debug 로그 활성");
    }

    match args.mode {
        ModeArg::Sim => run_sim_entry(&args)?,
        ModeArg::Real => run_real_entry(&args)?,
    }
    return Ok(());
}

fn run_sim_entry(args: &Args) -> Result<()> {
    let physics = PhysicsParams::default();
    let rail_frame = args.rail_frame();
    let intercept = args.intercept_window();
    let robot = robot_with_rail_frame(rail_frame).context("설정한 레일 위치로 로봇 조립")?;
    info!(
        mode = "sim",
        restitution = physics.restitution,
        "defaults SSOT"
    );
    let mut initial_controls = SimRuntimeControls::default();
    initial_controls.rail_frame = rail_frame;
    initial_controls.intercept = intercept;
    if args.ball_launch_x_m.is_some()
        || args.ball_launch_y_m.is_some()
        || args.ball_launch_z_m.is_some()
    {
        let muzzle = initial_controls.shooter.muzzle_position();
        initial_controls.shooter.set_muzzle_xyz(
            args.ball_launch_x_m.unwrap_or(f64::from(muzzle.x)),
            args.ball_launch_y_m.unwrap_or(f64::from(muzzle.y)),
            args.ball_launch_z_m.unwrap_or(f64::from(muzzle.z)),
        );
    }
    let controls = Arc::new(Mutex::new(initial_controls));
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
        world.set_intercept_window(intercept);
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

fn validate_setup_args(args: &Args) -> Result<()> {
    ensure!(
        args.table_distance_m.is_finite() && args.table_distance_m >= 0.0,
        "--table-distance-m은 0 이상의 유한한 값이어야 합니다"
    );
    ensure!(
        args.rail_bottom_z_m.is_finite() && args.rail_bottom_z_m >= 0.0,
        "--rail-bottom-z-m은 0 이상의 유한한 값이어야 합니다"
    );
    args.intercept_window().validate()?;
    for (name, value) in [
        ("--ball-launch-x-m", args.ball_launch_x_m),
        ("--ball-launch-y-m", args.ball_launch_y_m),
        ("--ball-launch-z-m", args.ball_launch_z_m),
    ] {
        ensure!(
            value.is_none_or(f64::is_finite),
            "{name}은 유한한 값이어야 합니다"
        );
    }
    return Ok(());
}

#[cfg(feature = "real")]
fn run_real_entry(args: &Args) -> Result<()> {
    return real::run(args);
}

#[cfg(not(feature = "real"))]
fn run_real_entry(_args: &Args) -> Result<()> {
    anyhow::bail!(
        "real 모드가 빌드에서 빠졌습니다 — `real` feature 없이 빌드된 바이너리입니다 \
         (기본 feature에 포함되어 있으니 `--no-default-features`를 쓰지 않았는지 확인하세요)"
    );
}
