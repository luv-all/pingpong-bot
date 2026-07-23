//! 각 축을 수동 구동해 배선·방향·한계를 검증 (plan §3.4).
//!
//! `Hardware` 포트를 런타임과 동일한 코드 경로로 사용한다.

use std::thread;
use std::time::Duration;

use anyhow::{Context, Result, bail, ensure};
use clap::Parser;
use pingpong_bot::{
    Hardware, Joints, RailMotion, RealHardware, SwingTrajectory, competition_dynamixel,
};

#[derive(Parser)]
#[command(name = "jog_axis", about = "축 수동 조그 (Hardware 포트)")]
struct Args {
    /// Dynamixel 시리얼 포트 (entry 기본 COM8 덮어씀).
    #[arg(long)]
    port: Option<String>,
    /// 0부터 시작하는 단축 인덱스.
    #[arg(long, requires = "angle_deg", conflicts_with = "angles_deg")]
    joint: Option<usize>,
    /// `--joint`에 지정한 축의 목표각 [deg].
    #[arg(long, requires = "joint", conflicts_with = "angles_deg")]
    angle_deg: Option<f64>,
    /// 모든 축의 URDF 관절각 [deg] (Dynamixel 절대각 아님), 예: `0,0,0,0`.
    #[arg(long, value_delimiter = ',', conflicts_with_all = ["joint", "angle_deg"])]
    angles_deg: Option<Vec<f64>>,
    /// 이동 시간 [s].
    #[arg(long, default_value_t = 1.0)]
    duration: f64,
    /// 현재각에서 허용할 축별 최대 이동량 [deg].
    #[arg(long, default_value_t = 15.0)]
    max_delta_deg: f64,
    /// 시리얼 포트를 열지 않고 변환·executor만 검증.
    #[arg(long)]
    dry_run: bool,
}

fn target_positions(args: &Args, present: &[f64]) -> Result<Vec<f64>> {
    ensure!(
        args.duration.is_finite() && args.duration > 0.0,
        "duration은 0보다 커야 합니다"
    );
    ensure!(
        args.max_delta_deg.is_finite() && args.max_delta_deg > 0.0,
        "max-delta-deg는 0보다 커야 합니다"
    );
    let target = if let Some(angles) = &args.angles_deg {
        ensure!(
            angles.len() == present.len(),
            "angles-deg는 {}개여야 합니다",
            present.len()
        );
        angles
            .iter()
            .map(|degrees| {
                ensure!(degrees.is_finite(), "목표각은 유한해야 합니다");
                Ok(degrees.to_radians())
            })
            .collect::<Result<Vec<_>>>()?
    } else if let (Some(joint), Some(angle_deg)) = (args.joint, args.angle_deg) {
        ensure!(joint < present.len(), "joint {joint}가 범위를 벗어났습니다");
        ensure!(angle_deg.is_finite(), "목표각은 유한해야 합니다");
        let mut target = present.to_vec();
        target[joint] = angle_deg.to_radians();
        target
    } else {
        bail!("--joint/--angle-deg 또는 --angles-deg를 지정하세요");
    };
    let max_delta = args.max_delta_deg.to_radians();
    for (index, (from, to)) in present.iter().zip(&target).enumerate() {
        ensure!(
            (to - from).abs() <= max_delta,
            "joint {index} 최대 이동량 {}° 초과 (요청 {:.1}°)",
            args.max_delta_deg,
            (to - from).abs().to_degrees()
        );
    }
    return Ok(target);
}

fn run(args: Args) -> Result<()> {
    let mut config = competition_dynamixel();
    if let Some(port) = &args.port {
        config.port = port.clone();
    }
    let mut hardware = if args.dry_run {
        RealHardware::dry_run(config, None)
    } else {
        RealHardware::new(config, None)
    }
    .context("Dynamixel 초기화 실패")?;

    let start = hardware.read_pose().context("현재 관절각 읽기 실패")?;
    let target = target_positions(&args, &start.joints.values)?;
    let joint_count = target.len();
    let trajectory = SwingTrajectory::new(
        start.joints,
        Joints::from_slice(&target),
        vec![0.0; joint_count],
        vec![0.0; joint_count],
        args.duration,
        RailMotion::fixed(0.0),
    );
    hardware.command(&trajectory).context("조그 명령 실패")?;
    while hardware.is_busy() {
        thread::sleep(Duration::from_millis(10));
    }
    let pose = hardware.read_pose().context("최종 관절각 읽기 실패")?;
    println!("joints_rad={:?}", pose.joints.values);
    return Ok(());
}

fn main() -> Result<()> {
    return run(Args::parse());
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::{Args, target_positions};

    #[test]
    fn single_joint_target_preserves_other_present_angles() {
        let args =
            Args::try_parse_from(["jog_axis", "--joint", "2", "--angle-deg", "5", "--dry-run"])
                .expect("args");

        let target = target_positions(&args, &[0.1, 0.2, 0.3, 0.4]).expect("target");
        assert_eq!(target[0], 0.1);
        assert_eq!(target[1], 0.2);
        assert!((target[2].to_degrees() - 5.0).abs() < 1e-9);
        assert_eq!(target[3], 0.4);
    }

    #[test]
    fn rejects_large_joint_jump() {
        let args = Args::try_parse_from(["jog_axis", "--angles-deg", "0,0,90,0", "--dry-run"])
            .expect("args");

        let error = target_positions(&args, &[0.0; 4]).unwrap_err();
        assert!(error.to_string().contains("최대 이동량"));
    }
}
