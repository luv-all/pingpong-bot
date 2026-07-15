//! 공 낙하 바운스 전후 속도비로 반발계수 e를 측정 (plan §3.4).
//!
//! 산출물: `config.toml` `[physics].restitution` / `drag` (기본 `config/default.toml`)

use std::fs;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{bail, Context, Result};
use clap::Parser;
use nalgebra::Vector3;
use pingpong_domain::constants::{ball, table, TABLE_BOUNCE_RESTITUTION};
use pingpong_domain::{
    drag_from_trajectory, merge_physics_into_config, physics_coeffs_toml,
    restitution_from_bounce_heights, restitution_from_normal_speeds, Arm, PhysicsConfig,
};
use pingpong_infra::{BallVec3, SimWorld};

#[derive(Parser, Debug)]
#[command(
    name = "measure_restitution",
    about = "반발계수 e 측정 → config [physics] 자동 반영. 항력은 --drag-csv"
)]
struct Args {
    /// 연속 바운스 정점 높이 [m] (쉼표)
    #[arg(long, value_name = "H0,H1,...")]
    heights: Option<String>,

    /// 법선 속력 쌍 |vin|:|vout| 목록 (쉼표)
    #[arg(long, value_name = "VIN:VOUT,...")]
    vz_pairs: Option<String>,

    /// Rapier sim 수직 낙하로 e 추정
    #[arg(long)]
    sim: bool,

    /// 탄도 적분으로 설정된 e 검증
    #[arg(long)]
    sim_ballistics: bool,

    /// 비행 궤적 CSV `t,x,y,z` → 항력 k 적합
    #[arg(long, value_name = "PATH")]
    drag_csv: Option<PathBuf>,

    /// 갱신할 런타임 config
    #[arg(long, value_name = "PATH", default_value = "config/default.toml")]
    config: PathBuf,

    /// config에 쓰지 않고 stdout 스니펫만
    #[arg(long)]
    dry_run: bool,

    /// sim 낙하 초기 높이 (테이블 면 위) [m]
    #[arg(long, default_value_t = 0.40)]
    drop_height: f64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut patch = PhysicsConfig::default();

    if let Some(ref csv) = args.drag_csv {
        let samples = load_traj_csv(csv)?;
        let k = drag_from_trajectory(&samples)
            .context("항력 적합 실패 — 샘플≥3, 비행 구간 속도≥0.3 m/s")?;
        println!("drag k = {k:.8}  (from {})", csv.display());
        patch.drag = Some(k);
    }

    if let Some(ref raw) = args.heights {
        let hs = parse_f64_list(raw)?;
        let e = restitution_from_bounce_heights(&hs)
            .context("높이로부터 e 추정 실패 — 높이 ≥2개, 양수")?;
        println!("restitution e = {e:.6}  (from {} heights)", hs.len());
        patch.restitution = Some(e);
    }

    if let Some(ref raw) = args.vz_pairs {
        let pairs = parse_pairs(raw)?;
        let e = restitution_from_normal_speeds(&pairs).context("속도 쌍으로부터 e 추정 실패")?;
        println!("restitution e = {e:.6}  (from {} vz pairs)", pairs.len());
        patch.restitution = Some(e);
    }

    if args.sim_ballistics {
        let e = measure_e_ballistics(args.drop_height)?;
        println!("restitution e = {e:.6}  (ballistics; configured={TABLE_BOUNCE_RESTITUTION})");
        patch.restitution = Some(e);
    }

    if args.sim {
        let e = measure_e_in_sim(args.drop_height)?;
        println!(
            "restitution e = {e:.6}  (sim drop; configured TABLE_BOUNCE_RESTITUTION={TABLE_BOUNCE_RESTITUTION})"
        );
        patch.restitution = Some(e);
    }

    if patch.is_empty() {
        bail!(
            "입력이 없습니다. 예:\n  \
             --heights 0.40,0.29,0.21\n  \
             --vz-pairs 2.0:1.7\n  \
             --sim\n  \
             --sim-ballistics\n  \
             --drag-csv traj.csv\n  \
             (기본 --config config/default.toml, --dry-run 으로 쓰기 생략)"
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

fn measure_e_ballistics(drop_height: f64) -> Result<f64> {
    use pingpong_domain::ballistics::semi_implicit_euler;

    let floor = table::SURFACE_Z + ball::RADIUS;
    let mut pos = Vector3::new(
        table::WIDTH_X * 0.5,
        table::LENGTH_Y * 0.5,
        floor + drop_height,
    );
    let mut vel = Vector3::zeros();
    let dt = 0.001;
    let mut vin = None;
    let mut vout = None;
    let mut prev_vz: f64 = 0.0;

    for _ in 0..10_000 {
        let (np, nv) = semi_implicit_euler(pos, vel, dt, 0.0);
        if vin.is_none() && prev_vz < -0.5 && nv.z >= 0.0 {
            vin = Some((-prev_vz).max(1e-6_f64));
            vout = Some(nv.z.max(0.0_f64));
            break;
        }
        prev_vz = nv.z;
        pos = np;
        vel = nv;
    }
    let (vin, vout) = match (vin, vout) {
        (Some(a), Some(b)) => (a, b),
        _ => bail!("ballistics 바운스를 잡지 못함"),
    };
    return restitution_from_normal_speeds(&[(vin, vout)]).context("ballistics e");
}

fn measure_e_in_sim(drop_height: f64) -> Result<f64> {
    let arm = Arc::new(Arm::competition().context("competition arm")?);
    let mut world = SimWorld::new(arm, None);
    world.set_use_ground_truth(false);

    let x = table::WIDTH_X * 0.5;
    let y = table::LENGTH_Y * 0.35;
    let z0 = table::SURFACE_Z + ball::RADIUS + drop_height;
    world.launch_ball_at(
        BallVec3::new(x as f32, y as f32, z0 as f32),
        BallVec3::new(0.0, 0.0, -0.01),
        BallVec3::new(0.0, 0.0, 0.0),
    );

    let dt = 1.0 / 1000.0;
    let mut min_vz = 0.0_f64;
    let mut max_vz_after = 0.0_f64;
    let mut saw_descent = false;
    let mut bounced = false;
    let floor = table::SURFACE_Z + ball::RADIUS;

    for _ in 0..8000 {
        world.step(dt, None);
        let z = f64::from(world.ball_position().z);
        let vz = f64::from(world.ball_velocity().z);
        if z < floor + 0.15 && vz < min_vz {
            min_vz = vz;
            saw_descent = true;
        }
        if saw_descent && vz > 0.05 {
            bounced = true;
            max_vz_after = max_vz_after.max(vz);
        }
        if bounced && vz < max_vz_after * 0.5 && max_vz_after > 0.1 {
            break;
        }
    }

    if !bounced || min_vz >= -0.1 {
        bail!(
            "sim 바운스 미검출 (min_vz={min_vz:.3}, max_vz_after={max_vz_after:.3}) — --sim-ballistics 사용"
        );
    }
    let vin = (-min_vz).abs();
    let vout = max_vz_after;
    println!("sim vz_in={vin:.4} vz_out={vout:.4}");
    return restitution_from_normal_speeds(&[(vin, vout)]).context("sim e 계산 실패");
}

fn parse_f64_list(raw: &str) -> Result<Vec<f64>> {
    let mut out = Vec::new();
    for part in raw.split(',') {
        let s = part.trim();
        if s.is_empty() {
            continue;
        }
        out.push(
            s.parse::<f64>()
                .with_context(|| format!("숫자 아님: {s}"))?,
        );
    }
    return Ok(out);
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

fn load_traj_csv(path: &PathBuf) -> Result<Vec<(f64, Vector3<f64>)>> {
    let text = fs::read_to_string(path).with_context(|| format!("읽기: {}", path.display()))?;
    let mut samples = Vec::new();
    for (line_no, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("t") {
            continue;
        }
        let cols: Vec<_> = line.split(|c| c == ',' || c == ' ' || c == '\t').collect();
        if cols.len() < 4 {
            bail!("{}:{} — t,x,y,z 필요", path.display(), line_no + 1);
        }
        let t: f64 = cols[0].parse().context("t")?;
        let x: f64 = cols[1].parse().context("x")?;
        let y: f64 = cols[2].parse().context("y")?;
        let z: f64 = cols[3].parse().context("z")?;
        samples.push((t, Vector3::new(x, y, z)));
    }
    return Ok(samples);
}
