//! YCrCb 등 색 공간 마스크로 공 검출 — `Detector` 포트 실험 (plan §5.3, §3.4).

use clap::Parser;

#[derive(Parser)]
#[command(name = "detect_colormask", about = "색상 마스크 검출 실험")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("색상 마스크 검출 (plan.md §3.4)");
}
