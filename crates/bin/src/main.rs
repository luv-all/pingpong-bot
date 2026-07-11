//! # pingpong-bot 런타임
//!
//! 최종 바이너리 진입점. CLI로 sim/real 모드를 선택하고
//! `infra` 어댑터를 골라 `app::run()`에 주입(DI)한다.
//!
//! ```bash
//! cargo run -p pingpong-bot -- --gui
//! cargo run -p pingpong-bot -- --frames 60 --sim-speed 5 --shoot-on-start
//! ```

use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use pingpong_app::{PipelineConfig, Robot, shared_competition_arm};
use pingpong_domain::{Arm, CameraId, HitPlane, constants::table};
use pingpong_infra::{
    RobotBuilder, SimBallEstimator, SimRuntimeControls, SimSession, SimSessionConfig,
    SimViewerOptions, TracingTelemetry, new_shutdown_flag, run_sim_viewer,
};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

/// CLI 실행 모드.
#[derive(Debug, Clone, Copy, ValueEnum)]
enum Mode {
    /// Rapier 디지털 트윈
    Sim,
    /// 실물 하드웨어 (Windows)
    Real,
}

/// CLI 인자.
#[derive(Parser)]
#[command(name = "pingpong-bot", about = "협력 랠리 핑퐁 로봇 런타임")]
struct Args {
    /// 실행 모드: sim(기본) 또는 real(Windows + feature 필요)
    #[arg(long, value_enum, default_value_t = Mode::Sim)]
    mode: Mode,

    /// headless sim (GUI 없이 N프레임 후 종료)
    #[arg(long)]
    no_gui: bool,

    /// 가상 카메라 프레임 수 (GUI 모드에서는 무시)
    #[arg(long, default_value_t = 300)]
    frames: u64,

    /// 접수 평면 y 좌표 [m] (로봇 앞 깊이)
    #[arg(long, default_value_t = table::DEFAULT_HIT_PLANE_Y)]
    hit_plane_y: f64,

    /// sim synthetic 카메라 대수 (2~N, 실험 후 Calibration으로 확정)
    #[arg(long, default_value_t = 3)]
    camera_count: u8,

    /// sim 시간 배율 (1.0 = 실시간, 10.0 = 10배속)
    #[arg(long, default_value_t = 1.0)]
    sim_speed: f64,

    /// sim 물리 적분 주파수 [Hz]
    #[arg(long, default_value_t = 1000.0)]
    physics_hz: f64,

    /// sim 가상 카메라 프레임률 [Hz]
    #[arg(long, default_value_t = 120.0)]
    frame_hz: f64,

    /// headless sim 시작 시 즉시 1회 발사
    #[arg(long, default_value_t = false)]
    shoot_on_start: bool,

    /// 로봇 URDF 파일 (미지정 시 내장 competition_arm)
    #[arg(long, value_name = "PATH")]
    urdf: Option<PathBuf>,

    /// URDF 엔드이펙터 link 이름 (미지정 시 체인 끝 link)
    #[arg(long, value_name = "LINK")]
    ee_link: Option<String>,

    /// 설정 파일 경로 (2단계에서 TOML 로드 — 카메라 대수·extrinsics 포함)
    #[arg(long)]
    config: Option<String>,
}

/// 프로그램 진입점.
fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("info".parse()?))
        .init();

    let args = Args::parse();

    if let Some(path) = &args.config {
        todo!("설정 파일 로드: {path}");
    }

    match args.mode {
        Mode::Sim => run_sim(args)?,
        Mode::Real => run_real()?,
    }

    return Ok(());
}

/// sim 모드: Rapier 디지털 트윈 + 슈터(+x) + 로봇(-x) + PassThrough 추정
fn run_sim(args: Args) -> Result<()> {
    if args.no_gui {
        warn!(
            frames = args.frames,
            "headless sim — GUI·슈터 패널 없음. `--shoot-on-start`로 자동 발사 가능"
        );
    } else {
        info!("sim kiss3d — 3D view + shooter panel (single window)");
    }

    let gui = !args.no_gui;
    let (arm, urdf) = load_robot(args.urdf.as_deref(), args.ee_link.as_deref())?;
    let controls = Arc::new(Mutex::new(SimRuntimeControls::default()));
    let shutdown = new_shutdown_flag();

    {
        let mut ctrl = controls.lock().expect("controls");
        ctrl.time_scale = args.sim_speed;
        if args.shoot_on_start && args.no_gui {
            ctrl.request_shoot();
        }
    }

    let mut session = SimSession::new(
        SimSessionConfig {
            physics_hz: args.physics_hz,
            frame_hz: args.frame_hz,
            time_scale: args.sim_speed,
            camera_count: args.camera_count,
        },
        Arc::clone(&arm),
        urdf.clone(),
        Arc::clone(&controls),
        Arc::clone(&shutdown),
    );

    {
        let world_arc = session.world();
        let mut world = world_arc.lock().expect("sim 월드");
        world.set_hit_plane(HitPlane {
            y: args.hit_plane_y,
        });
    }

    let frame_count = if gui { 0 } else { args.frames };

    let cameras: Vec<Box<dyn pingpong_domain::CameraSource>> = (0..args.camera_count)
        .map(|i| {
            Box::new(session.camera(CameraId::new(i), frame_count))
                as Box<dyn pingpong_domain::CameraSource>
        })
        .collect();

    let estimator = Box::new(SimBallEstimator::new(session.world()));
    let hardware = session.hardware();
    let telemetry = Arc::new(TracingTelemetry);
    let config = PipelineConfig {
        hit_plane: HitPlane {
            y: args.hit_plane_y,
        },
        control_hz: 100.0,
        arm,
    };

    if gui {
        let pipeline_shutdown = Arc::clone(&shutdown);
        let pipeline_handle = thread::spawn(move || {
            let result =
                pingpong_app::run(cameras, estimator, Box::new(hardware), config, telemetry);
            pipeline_shutdown.store(true, Ordering::Release);
            result
        });

        run_sim_viewer(SimViewerOptions {
            controls: session.controls(),
            world: session.world(),
            urdf,
            shutdown: Arc::clone(&shutdown),
        })
        .map_err(|e| anyhow::anyhow!("sim viewer: {e}"))?;

        shutdown.store(true, Ordering::Release);
        session.shutdown();
        pipeline_handle
            .join()
            .map_err(|_| anyhow::anyhow!("파이프라인 스레드 패닉"))?
            .context("sim 파이프라인 실행 실패")?;
    } else {
        pingpong_app::run(cameras, estimator, Box::new(hardware), config, telemetry)
            .context("sim 파이프라인 실행 실패")?;
        session.shutdown();
    }

    return Ok(());
}

/// real 모드: Dynamixel + AXL (2단계, Windows 전용).
fn run_real() -> Result<()> {
    todo!("real 하드웨어 모드 (Windows + `--features pingpong-infra/real`, plan.md §3.2)")
}

fn load_robot(
    urdf_path: Option<&std::path::Path>,
    ee_link: Option<&str>,
) -> Result<(Arc<Arm>, Option<Arc<pingpong_infra::UrdfRobot>>)> {
    let deployment = Robot::from_cli(
        urdf_path.map(std::path::Path::to_path_buf),
        ee_link.map(str::to_string),
    );
    let fallback = shared_competition_arm();

    if deployment.is_primitive() {
        return Ok((fallback, None));
    }

    let workspace = std::env::current_dir().context("현재 작업 디렉터리")?;
    let path = deployment
        .urdf_path(&workspace)
        .context("URDF 경로 해석 실패")?;
    let mount = deployment.mount();

    let built = RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(deployment.ee_link())
        .mount_xyz_rpy(mount.position, mount.rpy)
        .max_joint_speed(deployment.max_joint_speed())
        .build_with_arm_fallback(Arc::clone(&fallback))
        .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;

    if let Some(ref model) = built.urdf {
        info!(
            robot = %model.name,
            joints = model.joint_count(),
            ee = %model.ee_link,
            path = %path.display(),
            "URDF mesh 로드 — 제어·IK는 리니어 포함 competition arm 사용"
        );
    }

    // URDF는 kiss3d mesh 전용. plan_swing·리니어 X는 항상 competition primitive arm.
    return Ok((fallback, built.urdf));
}
