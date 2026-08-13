//! 관절 1축 왕복 스윕 + 실측 각속도 계산.

use std::io::{self, Write};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail, ensure};
use pingpong_bot::hardware::dynamixel::{DynamixelBus, DynamixelConfig};

use crate::args::Args;

/// 최근 이 개수의 표본이 이 각도[deg] 안에 다 들어오면 정지로 본다.
const SETTLE_WINDOW: usize = 4;
const SETTLE_TOLERANCE_DEG: f64 = 0.05;

pub fn run(args: &Args) -> Result<()> {
    let config = build_config(args);
    println!(
        "포트={} — 관절 {} 실측 속도 측정 시작 (카메라·레일·제어 없음)",
        config.port, args.joint
    );
    let mut bus = DynamixelBus::open(config).context("Dynamixel 버스 열기 실패")?;

    let start = bus.read_joints().context("시작 자세 읽기 실패")?;
    let joint_count = start.values.len();
    ensure!(
        args.joint < joint_count,
        "관절 인덱스 범위 초과: {} (관절 수 {joint_count})",
        args.joint
    );
    let start_angle = start.values[args.joint];
    let amplitude_rad = args.amplitude_deg.to_radians();
    let target_angle = start_angle + amplitude_rad;

    warn_and_confirm(args.joint, start_angle, target_angle)?;

    println!("이동 명령 전송...");
    let commanded_angle = bus
        .write_joint(args.joint, target_angle)
        .context("관절 명령 실패")?;
    let move_start = Instant::now();

    let samples = poll_until_settled(&mut bus, args, start_angle)?;

    report(&samples, start_angle, commanded_angle, move_start.elapsed());
    return Ok(());
}

fn build_config(args: &Args) -> DynamixelConfig {
    let mut config = DynamixelConfig::default();
    if let Some(port) = &args.dxl_port {
        config.port = port.clone();
    }
    return config;
}

fn warn_and_confirm(joint: usize, start_angle: f64, target_angle: f64) -> Result<()> {
    println!(
        "경고: 관절 {joint}를 {:.1}° → {:.1}°로 실제로 움직입니다 (최대 속도로 1회 이동).",
        start_angle.to_degrees(),
        target_angle.to_degrees()
    );
    println!("주변에 팔이 부딪힐 물체·사람이 없는지 확인하세요.");
    print!("계속하려면 y 를 입력하고 Enter, 취소하려면 다른 키를 입력하세요: ");
    io::stdout().flush().ok();
    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .context("확인 입력 읽기 실패")?;
    if input.trim().eq_ignore_ascii_case("y") {
        return Ok(());
    }
    bail!("사용자가 취소함 — 이동 명령을 보내지 않고 종료합니다");
}

/// `(elapsed_secs, angle_rad)` 표본 목록 — 정지로 판정되거나
/// `args.timeout_secs`에 도달하면 반환한다.
fn poll_until_settled(
    bus: &mut DynamixelBus,
    args: &Args,
    start_angle: f64,
) -> Result<Vec<(f64, f64)>> {
    let poll_period = Duration::from_secs_f64(1.0 / args.poll_hz.max(1.0));
    let timeout = Duration::from_secs_f64(args.timeout_secs.max(0.1));
    let move_start = Instant::now();
    let mut samples: Vec<(f64, f64)> = vec![(0.0, start_angle)];
    loop {
        thread::sleep(poll_period);
        let elapsed = move_start.elapsed();
        let joints = bus.read_joints().context("관절 폴링 실패")?;
        samples.push((elapsed.as_secs_f64(), joints.values[args.joint]));
        if elapsed >= timeout {
            println!("제한시간 도달 — 폴링 종료");
            break;
        }
        if samples.len() >= SETTLE_WINDOW {
            let recent = &samples[samples.len() - SETTLE_WINDOW..];
            let max = recent
                .iter()
                .map(|(_, a)| *a)
                .fold(f64::MIN, f64::max);
            let min = recent
                .iter()
                .map(|(_, a)| *a)
                .fold(f64::MAX, f64::min);
            if (max - min).to_degrees().abs() < SETTLE_TOLERANCE_DEG {
                break;
            }
        }
    }
    return Ok(samples);
}

fn report(samples: &[(f64, f64)], start_angle: f64, commanded_angle: f64, total_elapsed: Duration) {
    let mut peak_speed_rad_s = 0.0_f64;
    for window in samples.windows(2) {
        let (t0, a0) = window[0];
        let (t1, a1) = window[1];
        let dt = t1 - t0;
        if dt > 1e-6 {
            peak_speed_rad_s = peak_speed_rad_s.max(((a1 - a0) / dt).abs());
        }
    }
    let final_angle = samples.last().map_or(start_angle, |(_, a)| *a);
    let traveled_rad = (final_angle - start_angle).abs();
    let avg_speed_rad_s = if total_elapsed.as_secs_f64() > 1e-6 {
        traveled_rad / total_elapsed.as_secs_f64()
    } else {
        0.0
    };
    let rad_s_to_rpm = |v: f64| v * 60.0 / std::f64::consts::TAU;

    println!("\n=== 실측 결과 ===");
    println!(
        "목표 각도(한계로 잘렸을 수 있음)={:.2}°",
        commanded_angle.to_degrees()
    );
    println!(
        "실제 이동={:.2}° ({:.4}rad), 걸린 시간={:.3}s",
        traveled_rad.to_degrees(),
        traveled_rad,
        total_elapsed.as_secs_f64()
    );
    println!(
        "첨두 각속도(연속 표본 간 최대)={:.4} rad/s ({:.2} rpm)",
        peak_speed_rad_s,
        rad_s_to_rpm(peak_speed_rad_s)
    );
    println!(
        "평균 각속도(전체 구간)={:.4} rad/s ({:.2} rpm)",
        avg_speed_rad_s,
        rad_s_to_rpm(avg_speed_rad_s)
    );

    match pingpong_bot::defaults::robot() {
        Ok(active) => {
            let ceiling = active.arm.max_joint_speed;
            println!(
                "\n소프트웨어 설정 상한(arm.max_joint_speed)={:.4} rad/s ({:.2} rpm)",
                ceiling,
                rad_s_to_rpm(ceiling)
            );
            if peak_speed_rad_s > ceiling * 1.05 {
                println!(
                    "→ 실측 첨두({:.2} rpm)가 설정 상한({:.2} rpm)보다 뚜렷이 빠릅니다 — \
                     Velocity Limit 레지스터 변경이 실제로 여유를 늘렸을 가능성이 있습니다.",
                    rad_s_to_rpm(peak_speed_rad_s),
                    rad_s_to_rpm(ceiling)
                );
            } else {
                println!(
                    "→ 실측 첨두가 설정 상한과 비슷하거나 낮습니다 — 이 관절은 \
                     소프트웨어 상한(모터 데이터시트 무부하 속도 기반)에 이미 막혀 있고, \
                     Velocity Limit 레지스터가 병목이 아니었을 가능성이 있습니다."
                );
            }
        }
        Err(error) => {
            println!("\n(참고용 소프트웨어 상한 조회 실패: {error})");
        }
    }
}
