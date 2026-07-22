//! # pingpong-bot 런타임
//!
//! CLI로 sim/real 모드를 선택하고 파이프라인을 돌린다.
//!
//! ```bash
//! cargo run -p pingpong-bot
//! cargo run -p pingpong-bot -- config/experiment.toml
//! ```

mod config;

use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::{Context, Result};
use clap::Parser;
use pingpong_bot::{Arm, BallEkf, CameraId};
use pingpong_bot::{
    CameraFeed, RobotBuilder, SimRuntimeControls, SimSession, SimSessionConfig, SimViewerOptions,
    TracingTelemetry, new_shutdown_flag, run_sim_viewer,
};
#[cfg(feature = "real")]
use pingpong_bot::{Hardware, RealHardware};
use pingpong_bot::{PipelineConfig, find_robot, robot_ids_csv};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

use config::{DEFAULT_CONFIG_PATH, RuntimeConfig, RuntimeMode};
#[cfg(feature = "real")]
use pingpong_bot::VisionConfig;

/// CLI 인자.
#[derive(Parser)]
#[command(name = "pingpong-bot", about = "협력 랠리 핑퐁 로봇 런타임")]
struct Args {
    /// 런타임 TOML. 생략하면 config/default.toml.
    #[arg(value_name = "CONFIG", default_value = DEFAULT_CONFIG_PATH)]
    config: PathBuf,
}

/// 프로그램 진입점.
fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("info".parse()?))
        .init();

    let args = Args::parse();
    let runtime = RuntimeConfig::load(&args.config)?;
    let physics = runtime.physics;
    info!(
        path = %args.config.display(),
        mode = ?runtime.mode,
        cameras = runtime.camera_count,
        intercept_y_min = runtime.intercept.y_min,
        intercept_y_max = runtime.intercept.y_max,
        robot = %runtime.robot,
        restitution = physics.restitution,
        friction = physics.friction,
        drag = physics.drag,
        "설정 파일 로드"
    );

    match runtime.mode {
        RuntimeMode::Sim => run_sim(runtime)?,
        RuntimeMode::Real => run_real(runtime)?,
    }

    return Ok(());
}

/// sim 모드: Rapier 디지털 트윈 + 카메라→DLT→BallEkf→control 타격
fn run_sim(runtime: RuntimeConfig) -> Result<()> {
    let calibration = runtime.calibration()?;
    let physics = runtime.physics;
    if !runtime.sim.gui {
        warn!(
            frames = runtime.sim.frames,
            "headless sim — GUI·슈터 패널 없음. `sim.shoot_on_start`로 자동 발사 가능"
        );
    } else {
        info!("sim kiss3d — 3D view + shooter panel (single window)");
    }

    let gui = runtime.sim.gui;
    let robot_id = runtime.robot.as_str();
    let urdf_path = runtime.urdf_path();
    let (arm, urdf) = load_robot(robot_id, urdf_path.as_deref(), runtime.ee_link.as_deref())?;
    let controls = Arc::new(Mutex::new(SimRuntimeControls::default()));
    let shutdown = new_shutdown_flag();

    {
        let mut ctrl = controls.lock().expect("controls");
        ctrl.time_scale = runtime.sim.speed;
        if runtime.sim.shoot_on_start && !gui {
            ctrl.request_shoot();
        }
    }

    let mut session = SimSession::with_physics(
        SimSessionConfig {
            physics_hz: runtime.sim.physics_hz,
            frame_hz: runtime.sim.frame_hz,
            time_scale: runtime.sim.speed,
            camera_count: runtime.camera_count,
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
        world.set_intercept_window(runtime.intercept.into());
        world.set_use_ground_truth(runtime.sim.use_ground_truth);
        if runtime.sim.use_ground_truth {
            info!(
                "sim ground truth 타격 — EKF는 추정만, `sim.use_ground_truth=false`로 control 타격"
            );
        } else {
            warn!("EKF control 타격 모드 — 예측이 흔들리면 스윙이 이상해질 수 있음");
        }
    }

    let frame_count = if gui { 0 } else { runtime.sim.frames };

    let cameras: Vec<CameraFeed> = (0..runtime.camera_count)
        .map(|i| CameraFeed::Hint(Box::new(session.camera(CameraId::new(i), frame_count))))
        .collect();

    let estimator = Box::new(BallEkf::with_physics(physics));
    let hardware = session.hardware();
    let telemetry = Arc::new(TracingTelemetry);
    let config = PipelineConfig {
        intercept: runtime.intercept.into(),
        control_hz: 100.0,
        arm,
        calibration,
    };

    if gui {
        let pipeline_shutdown = Arc::clone(&shutdown);
        let pipeline_handle = thread::spawn(move || {
            let result =
                pingpong_bot::run(cameras, estimator, Box::new(hardware), config, telemetry);
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
        pingpong_bot::run(cameras, estimator, Box::new(hardware), config, telemetry)
            .context("sim 파이프라인 실행 실패")?;
        session.shutdown();
    }

    return Ok(());
}

/// real 모드: Dynamixel 스모크, `[vision]`이 있으면 캡처·검출 파이프라인.
#[cfg(feature = "real")]
fn run_real(runtime: RuntimeConfig) -> Result<()> {
    let dynamixel = runtime
        .hardware
        .dynamixel
        .clone()
        .context("mode=real에는 [hardware.dynamixel] 설정이 필요합니다")?;
    let rail = runtime.hardware.rail.clone();

    if !runtime.vision.cameras.is_empty() {
        let vision = runtime.vision.clone();
        return run_real_with_vision(runtime, dynamixel, rail, vision);
    }

    let mut hardware = RealHardware::new(dynamixel, rail).context("RealHardware 초기화 실패")?;
    let pose = hardware
        .read_pose()
        .context("Dynamixel 현재 관절 읽기 실패")?;
    info!(
        joints_rad = ?pose.joints.values,
        rail_x = pose.rail_x,
        "real 하드웨어 연결 스모크 완료 — vision.cameras 없으면 카메라 pipeline 생략"
    );
    return Ok(());
}

#[cfg(feature = "real")]
fn run_real_with_vision(
    runtime: RuntimeConfig,
    dynamixel: pingpong_bot::hardware::dynamixel::DynamixelConfig,
    rail: Option<pingpong_bot::hardware::rail::RailConfig>,
    vision: VisionConfig,
) -> Result<()> {
    use pingpong_bot::{CameraFeed, OpenCvCapture, TracingTelemetry, fuse_vision};

    let calibration = runtime.calibration().context("Calibration 로드")?;

    let mut feeds = Vec::new();
    for cam in &vision.cameras {
        let camera_id = CameraId::new(cam.id);
        let source: Box<dyn pingpong_bot::FrameSource> = if let Some(device) = cam.device {
            Box::new(
                OpenCvCapture::from_device(camera_id, device)
                    .map_err(anyhow::Error::msg)
                    .with_context(|| format!("카메라 device {device} 열기"))?,
            )
        } else {
            let path = cam.path.as_ref().context("path")?;
            let resolved = runtime.resolve_path(path);
            Box::new(
                OpenCvCapture::from_path(camera_id, &resolved)
                    .map_err(anyhow::Error::msg)
                    .with_context(|| format!("카메라 path {} 열기", resolved.display()))?,
            )
        };
        let params = calibration
            .params(camera_id)
            .cloned()
            .with_context(|| format!("{camera_id} Calibration 없음"))?;
        feeds.push(CameraFeed::Detect {
            source,
            detector: Box::new(fuse_vision(&vision).context("fuse_vision")?),
            params,
        });
    }

    let (arm, _) = load_robot(
        &runtime.robot,
        runtime.urdf_path().as_deref(),
        runtime.ee_link.as_deref(),
    )?;
    let hardware = RealHardware::new(dynamixel, rail).context("RealHardware 초기화 실패")?;
    let estimator = Box::new(BallEkf::with_physics(runtime.physics));
    let telemetry = Arc::new(TracingTelemetry);
    let config = PipelineConfig {
        intercept: runtime.intercept.into(),
        control_hz: 100.0,
        arm,
        calibration,
    };
    info!(
        cameras = feeds.len(),
        generators = ?vision.generators,
        "real 비전 파이프라인 시작"
    );
    pingpong_bot::run(feeds, estimator, Box::new(hardware), config, telemetry)
        .context("real 파이프라인 실행 실패")?;
    return Ok(());
}

#[cfg(not(feature = "real"))]
fn run_real(_runtime: RuntimeConfig) -> Result<()> {
    anyhow::bail!("real 모드는 `--features real`로 빌드해야 합니다 (Dynamixel 4축, AXL은 스텁).");
}

fn load_robot(
    robot_id: &str,
    urdf_path: Option<&std::path::Path>,
    ee_link: Option<&str>,
) -> Result<(Arc<Arm>, Option<Arc<pingpong_bot::UrdfRobot>>)> {
    // TOML `urdf_path`가 있으면 카탈로그 대신 해당 URDF를 제어·FK·IK에 직접 사용한다.
    if let Some(path) = urdf_path {
        let built = RobotBuilder::new()
            .urdf(path)
            .ee_link_opt(ee_link)
            .mount_preset(pingpong_bot::MountPreset::Rep103AtTableEnd)
            .max_joint_speed(2.5)
            .build()
            .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;
        info!(
            path = %path.display(),
            joints = built.arm.joint_count(),
            "커스텀 URDF — 제어·FK·IK·뷰어 모델 로드"
        );
        return Ok((built.arm, built.urdf));
    }

    let entry = find_robot(robot_id).ok_or_else(|| {
        anyhow::anyhow!(
            "알 수 없는 robot id `{robot_id}` — 사용 가능: {}",
            robot_ids_csv()
        )
    })?;
    let Some(rel) = entry.urdf_rel else {
        let arm = entry
            .primitive_arm()
            .ok_or_else(|| anyhow::anyhow!("robot `{robot_id}` primitive 빌더 누락"))?;
        info!(robot = robot_id, "빌더 프리셋 (URDF 없음)");
        return Ok((arm, None));
    };

    let workspace = std::env::current_dir().context("현재 작업 디렉터리")?;
    let path = workspace.join(rel);
    let built = RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(entry.ee_link)
        .mount_preset(pingpong_bot::MountPreset::Rep103AtTableEnd)
        .max_joint_speed(entry.max_joint_speed)
        .build()
        .with_context(|| format!("로봇 빌드 실패: {}", path.display()))?;

    if let Some(ref model) = built.urdf {
        info!(
            robot = robot_id,
            mesh = %model.name,
            joints = model.joint_count(),
            ee = %model.ee_link,
            path = %path.display(),
            "URDF 제어·FK·IK·뷰어 모델 로드"
        );
    }

    return Ok((built.arm, built.urdf));
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use clap::Parser;

    use super::*;

    #[test]
    fn cli_uses_default_toml_when_path_is_omitted() {
        let args = Args::try_parse_from(["pingpong-bot"]).expect("기본 CLI");
        assert_eq!(args.config, Path::new(config::DEFAULT_CONFIG_PATH));
    }

    #[test]
    fn cli_accepts_only_a_toml_path() {
        let args =
            Args::try_parse_from(["pingpong-bot", "config/experiment.toml"]).expect("설정 경로");
        assert_eq!(args.config, Path::new("config/experiment.toml"));
        assert!(Args::try_parse_from(["pingpong-bot", "--robot", "competition"]).is_err());
    }
}
