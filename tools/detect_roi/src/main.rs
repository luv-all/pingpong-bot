//! ROI 추적으로 공 검출·속도 측정 — infra vision 실험 (plan §5.3, §3.4).
//!
//! 전체 프레임 탐색 대신 직전 위치 주변만 처리해 120Hz를 유지한다.

use clap::Parser;

#[derive(Parser)]
#[command(name = "detect_roi", about = "ROI 추적 검출 실험")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("ROI 추적 검출 (plan.md §3.4)");
}
