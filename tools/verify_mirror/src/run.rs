//! 듀얼 MX-64 미러 페어 정렬 진단 루프.

use std::io::{self, Write};

use anyhow::{Context, Result, bail, ensure};
use pingpong_bot::hardware::dynamixel::{DynamixelBus, DynamixelConfig, MirrorAlignmentSample};

use crate::args::Args;

/// 정렬 검사가 구동을 막는 임계값과 같은 값 — `verify_mirror_alignment`의
/// `MIRROR_ALIGNMENT_MAX_ERROR_TICKS`를 그대로 옮겨 적었다. 라이브러리가
/// 이 상수를 공개하지 않으므로, 바뀌면 여기도 같이 바꿔야 한다.
const MIRROR_ALIGNMENT_MAX_ERROR_TICKS: i32 = 40;

pub fn run(args: &Args) -> Result<()> {
    let config = build_config(args);
    println!(
        "포트={} — 듀얼 MX-64 미러 정렬 진단 시작 (카메라·레일·제어 없음)",
        config.port
    );
    let mut bus = DynamixelBus::open(config).context("Dynamixel 버스 열기 실패")?;
    let mirror_ids = mirror_pair_ids(&bus);
    ensure!(
        !mirror_ids.is_empty(),
        "설정에 mirror_slaves가 없습니다 — 이 도구는 미러 페어가 있는 설정에서만 의미가 있습니다"
    );

    warn_and_confirm_torque_release(&mirror_ids)?;
    bus.diagnostic_set_torque_for_ids(&mirror_ids, false)
        .context("진단용 Torque Enable OFF 실패")?;
    println!("토크 OFF 완료 — 이제 손으로 관절을 돌려도 저항이 없어야 합니다.\n");

    let outcome = sample_loop(&mut bus);

    println!(
        "\n종료 — ID {mirror_ids:?} 토크는 계속 OFF 상태로 남습니다. \
         정상 구동 전에는 `--mode real`로 다시 시작하거나 전원을 재순환하세요."
    );
    return outcome;
}

fn build_config(args: &Args) -> DynamixelConfig {
    let mut config = DynamixelConfig::default();
    if let Some(port) = &args.dxl_port {
        config.port = port.clone();
    }
    return config;
}

fn mirror_pair_ids(bus: &DynamixelBus) -> Vec<u8> {
    let mut ids = Vec::new();
    for pair in &bus.config().mirror_slaves {
        if !ids.contains(&pair.master_id) {
            ids.push(pair.master_id);
        }
        if !ids.contains(&pair.slave_id) {
            ids.push(pair.slave_id);
        }
    }
    return ids;
}

fn warn_and_confirm_torque_release(mirror_ids: &[u8]) -> Result<()> {
    println!("경고: 아래 ID의 Torque Enable을 끕니다 — 해당 관절이 손으로 돌릴 수 있게 풀립니다.");
    println!("  대상 ID: {mirror_ids:?}");
    println!("  이 관절이 중력을 버티고 있었다면(팔이 아니라 순수 요(yaw) 회전이 아니라면),");
    println!("  토크를 끄는 순간 아래로 처질 수 있습니다 — 먼저 손으로 받칠 준비를 하세요.");
    print!("계속하려면 y 를 입력하고 Enter, 취소하려면 다른 키를 입력하세요: ");
    io::stdout().flush().ok();
    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .context("확인 입력 읽기 실패")?;
    if input.trim().eq_ignore_ascii_case("y") {
        return Ok(());
    }
    bail!("사용자가 취소함 — 토크를 끄지 않고 종료합니다");
}

fn sample_loop(bus: &mut DynamixelBus) -> Result<()> {
    let ticks_per_rev = f64::from(bus.config().ticks_per_revolution);
    let configured_offset = bus.config().mirror_slave_offset_ticks;
    loop {
        let samples = bus
            .read_mirror_alignment_samples()
            .context("미러 페어 실측 읽기 실패")?;
        for sample in &samples {
            print_sample(sample, ticks_per_rev, configured_offset);
        }
        print!("\nEnter = 다시 측정 (자세를 바꾼 뒤) · q + Enter = 종료: ");
        io::stdout().flush().ok();
        let mut input = String::new();
        io::stdin()
            .read_line(&mut input)
            .context("다음 측정 입력 읽기 실패")?;
        if input.trim().eq_ignore_ascii_case("q") {
            return Ok(());
        }
    }
}

fn print_sample(sample: &MirrorAlignmentSample, ticks_per_rev: f64, configured_offset: i32) {
    let deg = |tick: i32| f64::from(tick) * 360.0 / ticks_per_rev;
    let implied_offset = configured_offset + sample.error_ticks;
    let status = if sample.error_ticks.abs() > MIRROR_ALIGNMENT_MAX_ERROR_TICKS {
        "불일치 — 이 자세로 부팅하면 구동이 차단됩니다"
    } else {
        "정상 — 허용 범위 안"
    };
    println!(
        "[ID{}↔ID{}] master(ID{})={}tick({:.1}°)",
        sample.master_id, sample.slave_id, sample.master_id, sample.master_tick, deg(sample.master_tick)
    );
    println!(
        "  slave 현재(ID{})={}tick({:.1}°)   기대값={}tick({:.1}°)  (현 오프셋 상수 {}tick 기준)",
        sample.slave_id,
        sample.slave_tick,
        deg(sample.slave_tick),
        sample.expected_slave_tick,
        deg(sample.expected_slave_tick),
        configured_offset,
    );
    println!(
        "  오차={:+}tick({:+.1}°)  허용=±{}tick(±3.5°)  상태: {status}",
        sample.error_ticks,
        deg(sample.error_ticks),
        MIRROR_ALIGNMENT_MAX_ERROR_TICKS,
    );
    println!(
        "  → 정렬하려면 ID{}를 {}tick({:.1}°) → {}tick({:.1}°)로 옮기세요 ({:+}tick / {:+.1}°).",
        sample.slave_id,
        sample.slave_tick,
        deg(sample.slave_tick),
        sample.expected_slave_tick,
        deg(sample.expected_slave_tick),
        sample.expected_slave_tick - sample.slave_tick,
        deg(sample.expected_slave_tick - sample.slave_tick),
    );
    println!(
        "     손으로 돌려 위 '오차'가 0에 가까워지는 방향이 맞는 방향입니다."
    );
    println!(
        "  → 이 오차가 자세를 바꿔도 항상 비슷하게 나오면, 조립 오프셋 상수를 \
         {}tick → {}tick로 재보정하세요. 자세마다 오차가 크게 달라지면 상수 문제가 \
         아니라 백래시/헐거운 혼 등 기계적 문제를 먼저 점검하세요.\n",
        configured_offset, implied_offset,
    );
}
