//! 테이블 위 롤로 마찰계수 μ를 측정 (plan §3.4).
//!
//! 산출물: `config.toml` `[physics].friction` (기본 `config/default.toml`)

mod capture_loop;

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result, bail};
use clap::Parser;
use pingpong_bot::constants::{TABLE_BOUNCE_FRICTION, ball, table};
use pingpong_bot::{
    Arm, DEFAULT_CONFIG_PATH, DetectorKind, PhysicsConfig, friction_from_tangential_speeds,
    merge_physics_into_config, physics_coeffs_toml, resolve_calibration_path,
};
use pingpong_bot::{BallVec3, SimWorld};

#[derive(Parser, Debug)]
#[command(
    name = "measure_friction",
    about = "테이블 마찰 μ 측정 → config [physics]. 영상 멀티캠 또는 수동 숫자"
)]
struct Args {
    /// Calibration JSON. 생략 시 --config 의 calibration_path
    #[arg(long, value_name = "PATH")]
    calibration: Option<PathBuf>,
    #[arg(long = "video", value_name = "PATH")]
    videos: Vec<PathBuf>,
    #[arg(long = "device", value_name = "N")]
    devices: Vec<i32>,
    #[arg(long, default_value = "colormask")]
    detector: String,
    #[arg(long)]
    no_preview: bool,
    #[arg(long, default_value_t = 33)]
    wait_ms: i32,
    #[arg(long, default_value_t = 10_000)]
    max_frames: usize,
    #[arg(long)]
    fps: Option<f64>,
    #[arg(long, value_name = "VIN:VOUT,...")]
    vt_pairs: Option<String>,
    #[arg(long)]
    sim: bool,
    /// 런타임 TOML (calibration_path · [physics] merge)
    #[arg(long, value_name = "PATH", default_value = DEFAULT_CONFIG_PATH)]
    config: PathBuf,
    #[arg(long)]
    dry_run: bool,
    #[arg(long, default_value_t = 2.0)]
    horiz_speed: f64,
    #[arg(long, default_value_t = 0.25)]
    drop_height: f64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut patch = PhysicsConfig::default();

    if args.calibration.is_some() || !args.videos.is_empty() || !args.devices.is_empty() {
        let cal = resolve_calibration_path(&args.config, args.calibration.clone())
            .map_err(anyhow::Error::msg)?;
        let kind = DetectorKind::parse(&args.detector)
            .with_context(|| format!("unknown detector: {}", args.detector))?;
        let result = capture_loop::run_capture(
            &cal,
            &args.videos,
            &args.devices,
            kind,
            !args.no_preview,
            args.wait_ms,
            args.max_frames,
            args.fps,
        )?;
        for (i, r) in result.rolls.iter().enumerate() {
            println!(
                "roll[{i}] μ={:.4}  vt_in={:.3} vt_out={:.3}",
                r.mu, r.vt_in, r.vt_out
            );
        }
        let mu = result.mu.context("롤 구간을 찾지 못함")?;
        println!(
            "friction μ = {mu:.6}  (from {} rolls, traj={})",
            result.rolls.len(),
            result.traj.len()
        );
        patch.friction = Some(mu);
    }

    if let Some(ref raw) = args.vt_pairs {
        let pairs = parse_pairs(raw)?;
        let mu = friction_from_tangential_speeds(&pairs).context("접선 쌍으로부터 μ 추정 실패")?;
        println!("friction μ = {mu:.6}  (from {} vt pairs)", pairs.len());
        patch.friction = Some(mu);
    }

    if args.sim {
        let mu = measure_mu_in_sim(args.drop_height, args.horiz_speed)?;
        println!(
            "friction μ = {mu:.6}  (sim; configured TABLE_BOUNCE_FRICTION={TABLE_BOUNCE_FRICTION})"
        );
        patch.friction = Some(mu);
    }

    if patch.is_empty() {
        bail!(
            "입력이 없습니다. 예:\n  \
             --device 0 --device 1   # calibration_path 는 --config TOML\n  \
             --calibration calib.json --video cam0.mp4 --video cam1.mp4\n  \
             --vt-pairs 2.0:1.4\n  \
             --sim"
        );
    }

    if args.dry_run {
        print!(
            "{}",
            physics_coeffs_toml(patch.restitution, patch.friction, patch.drag)
        );
        return Ok(());
    }

    let merged = merge_physics_into_config(&args.config, &patch)
        .with_context(|| format!("config 갱신 실패: {}", args.config.display()))?;
    println!(
        "updated {} [physics] restitution={:?} friction={:?} drag={:?}",
        args.config.display(),
        merged.restitution,
        merged.friction,
        merged.drag
    );
    return Ok(());
}

fn measure_mu_in_sim(drop_height: f64, horiz_speed: f64) -> Result<f64> {
    let arm = Arc::new(Arm::competition().context("competition arm")?);
    let mut world = SimWorld::new(arm, None);
    world.set_use_ground_truth(false);

    let x = table::WIDTH_X * 0.5;
    let y = table::LENGTH_Y * 0.35;
    let z0 = table::SURFACE_Z + ball::RADIUS + drop_height;
    world.launch_ball_at(
        BallVec3::new(x as f32, y as f32, z0 as f32),
        BallVec3::new(horiz_speed as f32, 0.0, -0.01),
        BallVec3::new(0.0, 0.0, 0.0),
    );

    let dt = 1.0 / 1000.0;
    let mut min_vz = 0.0_f64;
    let mut vin_t = 0.0_f64;
    let mut vout_t = 0.0_f64;
    let mut saw_descent = false;
    let mut bounced = false;
    let floor = table::SURFACE_Z + ball::RADIUS;

    for _ in 0..8000 {
        world.step(dt, None);
        let pos = world.ball_position();
        let vel = world.ball_velocity();
        let z = f64::from(pos.z);
        let vz = f64::from(vel.z);
        let vt = (f64::from(vel.x).powi(2) + f64::from(vel.y).powi(2)).sqrt();

        if z < floor + 0.15 && vz < min_vz {
            min_vz = vz;
            vin_t = vt;
            saw_descent = true;
        }
        if saw_descent && vz > 0.05 {
            if !bounced {
                vout_t = vt;
            }
            bounced = true;
            if vz < 0.2 && vout_t > 0.0 {
                break;
            }
        }
    }

    if !bounced || vin_t < 1e-3 {
        bail!("sim 바운스 접선 속도 미검출 — --vt-pairs 로 수동 입력");
    }
    println!("sim vt_in={vin_t:.4} vt_out={vout_t:.4}");
    return friction_from_tangential_speeds(&[(vin_t, vout_t)]).context("sim μ 계산 실패");
}

fn parse_pairs(raw: &str) -> Result<Vec<(f64, f64)>> {
    let mut out = Vec::new();
    for part in raw.split(',') {
        let s = part.trim();
        if s.is_empty() {
            continue;
        }
        let (a, b) = s
            .split_once(':')
            .with_context(|| format!("VIN:VOUT 형식 필요: {s}"))?;
        out.push((
            a.trim().parse().with_context(|| format!("vin: {a}"))?,
            b.trim().parse().with_context(|| format!("vout: {b}"))?,
        ));
    }
    return Ok(out);
}
