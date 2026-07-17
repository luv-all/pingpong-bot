//! contour + 원형도(circularity) 게이팅으로 공 검출 — infra vision 실험 (plan §5.3).

use clap::Parser;

#[derive(Parser)]
#[command(name = "detect_contour", about = "contour 형상 게이팅 검출 실험")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("contour 형상 게이팅 검출 (plan.md §3.4)");
}
