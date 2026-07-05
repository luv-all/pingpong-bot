//! 공 낙하 바운스 전후 속도비로 반발계수 e를 측정 (plan §3.4).
//!
//! 산출물: e → Config(TOML) → EKF 바운스 식 (§6.1)

use clap::Parser;

#[derive(Parser)]
#[command(name = "measure_restitution", about = "반발계수 측정")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("반발계수 측정 (plan.md §3.4)");
}
