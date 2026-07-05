//! 글로벌 셔터로 비행 중인 공을 촬영·저장 — 검출/EKF 튜닝용 데이터셋 (plan §3.4).

use clap::Parser;

#[derive(Parser)]
#[command(name = "capture_flying_ball", about = "비행 공 캡처 데이터셋")]
struct Args {
    #[arg(long, default_value_t = 100)]
    frames: u64,
}

fn main() {
    let _args = Args::parse();
    todo!("비행 공 캡처 데이터셋 (plan.md §3.4)");
}
