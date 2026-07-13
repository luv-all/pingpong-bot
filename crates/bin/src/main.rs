//! # pingpong-bot 런타임
//!
//! 최종 바이너리 진입점. CLI로 sim/real 모드를 선택하고
//! `infra` 어댑터를 골라 `app::run()`에 주입(DI)한다.
//!
//! ```bash
//! cargo run -p pingpong-bot -- --gui
//! cargo run -p pingpong-bot -- --frames 60 --sim-speed 5 --shoot-on-start
//! ```

mod config;

use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use pingpong_app::{find_robot, robot_ids_csv, PipelineConfig, DEFAULT_ROBOT_ID};
use pingpong_domain::{Arm, BallEkf, Calibration, CameraId, HitPlane, constants::table};
use pingpong_infra::{
    RobotBuilder, SimRuntimeControls, SimSession, SimSessionConfig, SimViewerOptions,
    TracingTelemetry, new_shutdown_flag, run_sim_viewer,
};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

use config::RuntimeConfig;

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

    /// 로봇 프리셋 id (`pingpong_app::ROBOTS`, 미지정 시 config → competition)
    #[arg(long, value_name = "ID")]
    robot: Option<String>,

    /// 로봇 URDF 파일 (지정 시 `--robot` / config robot 무시)
    #[arg(long, value_name = "PATH")]
    urdf: Option<PathBuf>,

    /// URDF 엔드이펙터 link 이름 (미지정 시 체인 끝 link)
    #[arg(long, value_name = "LINK")]
    ee_link: Option<String>,

    /// 설정 파일 (TOML: hit_plane_y, camera_count, robot, calibration_path)
    #[arg(long, value_name = "PATH")]
    config: Option<String>,

    /// EKF→control로 타격 (기본은 sim 오라클=진실 상태 타격)
    #[arg(long, default_value_t = false)]
    ekf_swing: bool,
}

/// 프로그램 진입점.
fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("info".parse()?))
        .init();

    let mut args = Args::parse();
    let mut robot_id = pingpong_app::DEFAULT_ROBOT_ID.to_string();
    let mut calibration = Calibration::sim(args.camera_count);

    if let Some(path) = &args.config {
        let runtime = RuntimeConfig::load(std::path::Path::new(path))?;
        args.hit_plane_y = runtime.hit_plane_y;
        args.camera_count = runtime.camera_count;
        robot_id = runtime.robot.clone();
        calibration = runtime.calibration()?;
        info!(
            path,
            cameras = calibration.camera_count(),
            hit_plane_y = args.hit_plane_y,
            robot = %robot_id,
            "설정 파일 로드"
        );
    }

    // CLI `--robot`이 TOML보다 우선
    if let Some(id) = args.robot.take() {
        robot_id = id;
    }
    args.robot = Some(robot_id);

    match args.mode {
        Mode::Sim => run_sim(args, calibration)?,
        Mode::Real => run_real()?,
    }

    return Ok(());
}

/// sim 모드: Rapier 디지털 트윈 + 카메라→DLT→BallEkf→control 타격
fn run_sim(args: Args, calibration: Calibration) -> Result<()> {
    if args.no_gui {
        warn!(
            frames = args.frames,
            "headless sim — GUI·슈터 패널 없음. `--shoot-on-start`로 자동 발사 가능"
        );
    } else {
        info!("sim kiss3d — 3D view + shooter panel (single window)");
    }

    let gui = !args.no_gui;
    let robot_id = args
        .robot
        .as_deref()
        .unwrap_or(pingpong_app::DEFAULT_ROBOT_ID);
    let (arm, urdf, joint_map) =
        load_robot(robot_id, args.urdf.as_deref(), args.ee_link.as_deref())?;
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
        world.set_control_to_urdf(joint_map);
        world.set_hit_plane(HitPlane {
            y: args.hit_plane_y,
        });
        // 기본: 오라클 타격. EKF control은 아직 불안정해 팔을 헛스윙시킴.
        world.set_oracle_auto_swing(!args.ekf_swing);
        if args.ekf_swing {
            warn!("EKF control 타격 모드 — 예측이 흔들리면 스윙이 이상해질 수 있음");
        } else {
            info!("sim 오라클 타격 (진실 상태) — EKF는 추정만, `--ekf-swing`으로 control 타격");
        }
    }

    let frame_count = if gui { 0 } else { args.frames };

    let cameras: Vec<Box<dyn pingpong_domain::CameraSource>> = (0..args.camera_count)
        .map(|i| {
            Box::new(session.camera(CameraId::new(i), frame_count))
                as Box<dyn pingpong_domain::CameraSource>
        })
        .collect();

    let estimator = Box::new(BallEkf::new(0.0));
    let hardware = session.hardware();
    let telemetry = Arc::new(TracingTelemetry);
    let config = PipelineConfig {
        hit_plane: HitPlane {
            y: args.hit_plane_y,
        },
        control_hz: 100.0,
        arm,
        calibration,
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

/// real 모드: Dynamixel + AXL (마일스톤 5, Windows 전용).
fn run_real() -> Result<()> {
    anyhow::bail!(
        "real 모드는 아직 미구현입니다. Windows에서 `--features pingpong-infra/real` + Dynamixel/AXL 연동 후 사용 (plan.md §3.2)."
    );
}

fn load_robot(
    robot_id: &str,
    urdf_path: Option<&std::path::Path>,
    ee_link: Option<&str>,
) -> Result<(
    Arc<Arm>,
    Option<Arc<pingpong_infra::UrdfRobot>>,
    Option<Vec<Option<usize>>>,
)> {
    // `--urdf`가 있으면 카탈로그 무시, 제어 Arm만 기본 프리셋 (매핑 = truncate)
    if let Some(path) = urdf_path {
        let arm = find_robot(DEFAULT_ROBOT_ID)
            .expect("DEFAULT_ROBOT_ID")
            .arm();
        let built = RobotBuilder::new()
            .urdf(path)
            .ee_link_opt(ee_link)
            .mount_preset(pingpong_infra::MountPreset::Rep103AtTableEnd)
            .max_joint_speed(2.5)
            .build_with_arm_fallback(Arc::clone(&arm))
            .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;
        info!(path = %path.display(), "커스텀 URDF — 제어는 기본 빌더, 매핑 truncate");
        return Ok((arm, built.urdf, None));
    }

    let entry = find_robot(robot_id).ok_or_else(|| {
        anyhow::anyhow!("알 수 없는 robot id `{robot_id}` — 사용 가능: {}", robot_ids_csv())
    })?;
    let arm = entry.arm();
    let joint_map = entry.control_to_urdf_owned();

    let Some(rel) = entry.urdf_rel else {
        info!(robot = robot_id, "빌더 프리셋 (URDF 없음)");
        return Ok((arm, None, None));
    };

    let workspace = std::env::current_dir().context("현재 작업 디렉터리")?;
    let path = workspace.join(rel);
    let built = RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(entry.ee_link)
        .mount_preset(pingpong_infra::MountPreset::Rep103AtTableEnd)
        .max_joint_speed(entry.max_joint_speed)
        .build_with_arm_fallback(Arc::clone(&arm))
        .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;

    if let Some(ref model) = built.urdf {
        if let Some(ref map) = joint_map {
            pingpong_infra::validate_control_to_urdf_map(
                map,
                model.joint_count(),
                arm.joint_count(),
            )
            .map_err(|e| anyhow::anyhow!("robot `{robot_id}` 관절 매핑: {e}"))?;
        }
        info!(
            robot = robot_id,
            mesh = %model.name,
            joints = model.joint_count(),
            control_joints = arm.joint_count(),
            mapped = joint_map.is_some(),
            ee = %model.ee_link,
            path = %path.display(),
            "URDF mesh 로드 — 제어·IK는 카탈로그 빌더"
        );
    }

    return Ok((arm, built.urdf, joint_map));
}
