//! 배경 차분(background subtraction)만으로 공 검출 — `Detector` 포트 실험 (plan §3.4).
//!
//! 이긴 구현은 infra에 그대로 꽂는다 (코드 이동 없음).

use clap::Parser;

#[derive(Parser)]
#[command(name = "detect_bgsub", about = "배경 차분 검출 실험")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("배경 차분 검출 (plan.md §3.4)");
}
