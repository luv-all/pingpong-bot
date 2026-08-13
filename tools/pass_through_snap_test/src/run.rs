//! 실기 연결·확인·스트리밍·리포트.

use std::io::{self, Write};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use pingpong_bot::Point3;
use pingpong_bot::hardware::dynamixel::{DynamixelBus, DynamixelConfig};
use pingpong_bot::robot::Joints;

use crate::args::Args;
use crate::plan::SwingPlan;

pub fn run(args: &Args) -> Result<()> {
    let config = build_config(args);
    println!("포트={} — pass-through/wrist-snap 격리 테스트 시작", config.port);
    let mut bus = DynamixelBus::open(config).context("Dynamixel 버스 열기 실패")?;

    let active = pingpong_bot::defaults::robot().context("소프트웨어 팔 모델 로드 실패")?;
    let arm = &active.arm;

    let current = bus.read_joints().context("현재 관절각 읽기 실패")?;
    let target = Point3::new(args.target_x, args.target_y, args.target_z);

    let plan = SwingPlan::build(
        arm,
        &current,
        target,
        args.overshoot_m,
        args.total_duration_secs,
        args.impact_time_secs,
        args.wrist_cocked_deg.to_radians(),
        args.backswing_duration_secs,
        args.ramp_secs,
        args.snap_velocity_margin,
    )
    .map_err(|error| anyhow::anyhow!("계획 실패: {error}"))?;

    print_summary(&current, &plan);
    confirm()?;

    let samples = stream_and_record(&mut bus, &plan, args.poll_hz)?;
    report(&samples, &current, args.impact_time_secs);
    return Ok(());
}

fn build_config(args: &Args) -> DynamixelConfig {
    let mut config = DynamixelConfig::default();
    if let Some(port) = &args.dxl_port {
        config.port = port.clone();
    }
    return config;
}

fn print_summary(current: &Joints, plan: &SwingPlan) {
    let overshoot = plan.overshoot_joints();
    println!("\n=== 계획 ===");
    for index in 0..4 {
        println!(
            "  j{index}: {:.2}° -> {:.2}°",
            current.values[index].to_degrees(),
            overshoot.values[index].to_degrees()
        );
    }
    println!(
        "  손목 스냅 목표각={:.2}°, 전 구간 첨두 각속도(참고용)={:.4} rad/s",
        plan.wrist_snap_target_angle().to_degrees(),
        plan.wrist_peak_speed(50)
    );
    println!("  총 소요 시간={:.3}s", plan.total_duration_secs());
}

fn confirm() -> Result<()> {
    println!("\n경고: 위 계획대로 4관절을 실제로 동시에 움직입니다.");
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

fn stream_and_record(
    bus: &mut DynamixelBus,
    plan: &SwingPlan,
    poll_hz: f64,
) -> Result<Vec<(f64, Joints)>> {
    let poll_period = Duration::from_secs_f64(1.0 / poll_hz.max(1.0));
    let total_duration = Duration::from_secs_f64(plan.total_duration_secs());
    let start = Instant::now();
    let mut samples = Vec::new();
    loop {
        let elapsed = start.elapsed();
        let target = plan.sample(elapsed.as_secs_f64());
        bus.write_joints(&target).context("스트리밍 명령 실패")?;
        let measured = bus.read_joints().context("스트리밍 중 관절각 읽기 실패")?;
        samples.push((elapsed.as_secs_f64(), measured));
        if elapsed >= total_duration {
            break;
        }
        thread::sleep(poll_period);
    }
    return Ok(samples);
}

fn report(samples: &[(f64, Joints)], start: &Joints, impact_time_secs: f64) {
    println!("\n=== 실측 결과 ===");
    let mut peak_speed = [0.0_f64; 4];
    for window in samples.windows(2) {
        let (t0, q0) = &window[0];
        let (t1, q1) = &window[1];
        let dt = t1 - t0;
        if dt > 1e-6 {
            for index in 0..4 {
                let speed = (q1.values[index] - q0.values[index]).abs() / dt;
                peak_speed[index] = peak_speed[index].max(speed);
            }
        }
    }
    for index in 0..4 {
        println!(
            "  j{index}: 시작={:.2}° 첨두 각속도={:.4} rad/s",
            start.values[index].to_degrees(),
            peak_speed[index]
        );
    }

    let closest = samples
        .iter()
        .min_by(|(t_a, _), (t_b, _)| (t_a - impact_time_secs).abs().total_cmp(&(t_b - impact_time_secs).abs()));
    if let Some((t, joints)) = closest {
        println!(
            "\n임팩트 추정 시각({impact_time_secs:.3}s)에 가장 가까운 실측 표본(t={t:.3}s):"
        );
        for (index, value) in joints.values.iter().enumerate() {
            println!("  j{index}={:.2}°", value.to_degrees());
        }
    }
}
