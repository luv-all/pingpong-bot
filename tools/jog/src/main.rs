//! 인터랙티브 조그 GUI — sim 미리보기 후 Sync/Apply로 실기 전송.
//!
//! planner(`plan_swing`)는 쓰지 않는다. 목표 pose → quintic → `robot::Handle::play` /
//! Apply 시 `Hardware::command`.

mod args;
mod jog_ui;
mod motion;
mod panel;
mod state;

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_bot::defaults::robot;
use pingpong_bot::hardware::RealHardware;
use pingpong_bot::hardware::dynamixel::DynamixelConfig;
use pingpong_bot::hardware::rail::RailConfig;
use pingpong_bot::logging::init_tracing;
use pingpong_bot::sim::gui::{SceneUiHook, SimScene};
use pingpong_bot::sim::session::{SimRuntimeControls, SimSession, SimSessionConfig};
use tracing::info;

use args::Args;
use jog_ui::JogUi;
use state::JogApp;

fn main() -> Result<()> {
    let args = Args::parse();
    init_tracing(args.debug, &["jog", "pingpong_bot"]);
    if args.debug {
        info!("debug 로그 활성 — Dynamixel 재시도·AXL API 실패 code가 출력됩니다");
    }
    return run(args);
}

fn run(args: Args) -> Result<()> {
    let mut dxl = DynamixelConfig::default();
    if let Some(port) = &args.port {
        dxl.port = port.clone();
    }
    let mut rail_cfg = RailConfig::default();
    if let Some(dll_path) = args.dll_path {
        rail_cfg.dll_path = dll_path;
    }

    if args.debug {
        info!(
            port = %dxl.port,
            baudrate = dxl.baudrate,
            motor_ids = ?dxl.motor_ids,
            dry_run = args.dry_run,
            "Dynamixel 설정"
        );
        info!(
            enabled = rail_cfg.enabled,
            dll = %rail_cfg.dll_path.display(),
            axis = rail_cfg.axis,
            irq_no = rail_cfg.irq_no,
            reverse = rail_cfg.reverse,
            x_min_m = rail_cfg.x_min_m,
            x_max_m = rail_cfg.x_max_m,
            vel = rail_cfg.vel,
            "레일 설정"
        );
    }

    let robot = robot().context("defaults::robot")?;
    let hardware = if args.dry_run {
        RealHardware::dry_run_with_arm(dxl, Some(rail_cfg), Arc::clone(&robot.arm))
    } else {
        RealHardware::new(dxl, Some(rail_cfg), Arc::clone(&robot.arm))
    }
    .context("하드웨어 초기화 실패")?;
    let hardware = Arc::new(Mutex::new(hardware));

    let shutdown = SimRuntimeControls::new_shutdown();
    let controls = Arc::new(Mutex::new(SimRuntimeControls::default()));
    let mut session = SimSession::new(
        SimSessionConfig {
            camera_count: 0,
            ..SimSessionConfig::default()
        },
        robot.clone(),
        Arc::clone(&controls),
        Arc::clone(&shutdown),
    );
    {
        let world = session.world();
        let mut world = world.lock().expect("sim 월드");
        world.set_use_ground_truth(false);
        world.robot_mut().set_auto_return_to_center(false);
        world.set_kinematic_robot(true);
    }

    let app = Arc::new(Mutex::new(JogApp::new(
        Arc::clone(&robot.arm),
        Arc::clone(&hardware),
        args.dry_run,
    )));

    let ui_hook: SceneUiHook = Arc::new(Mutex::new(JogUi {
        app: Arc::clone(&app),
    }));

    let world = session.world();
    let scene = SimScene::builder()
        .title(if args.dry_run { "jog (dry-run)" } else { "jog" })
        .with_robot(Arc::clone(&world))
        .with_ball()
        .ghost_ball(true)
        .urdf(robot.urdf.clone())
        .with_ui_hook(ui_hook)
        .build();

    let robot_handle = scene
        .robot()
        .cloned()
        .ok_or_else(|| anyhow::anyhow!("robot layer missing"))?;
    let ball_handle = scene
        .ball()
        .cloned()
        .ok_or_else(|| anyhow::anyhow!("ball layer missing"))?;
    {
        let mut app = app.lock().expect("jog app");
        app.attach_robot(robot_handle);
        app.attach_ball(ball_handle);
        if let Err(err) = app.sync() {
            anyhow::bail!("boot sync 실패: {err:#}");
        }
    }

    info!(
        dry_run = args.dry_run,
        "jog GUI — Sync / Preview / Apply / Discard"
    );
    scene
        .run(Arc::clone(&shutdown))
        .map_err(|e| anyhow::anyhow!(e))?;
    session.shutdown();
    return Ok(());
}
