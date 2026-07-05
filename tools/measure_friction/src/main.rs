//! 접선 속도 변화로 마찰계수 μ를 측정 (plan §3.4).
//!
//! 산출물: μ → Config(TOML) → EKF 바운스 식 (§6.1)

use clap::Parser;

#[derive(Parser)]
#[command(name = "measure_friction", about = "마찰계수 측정")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("마찰계수 측정 (plan.md §3.4)");
}
