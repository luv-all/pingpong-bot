//! 각 축을 수동 구동해 배선·방향·한계를 검증 (plan §3.4).
//!
//! `Hardware` 포트를 런타임과 동일한 코드 경로로 사용한다.

use clap::Parser;

#[derive(Parser)]
#[command(name = "jog_axis", about = "축 수동 조그 (Hardware 포트)")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("축 수동 조그 (plan.md §3.4)");
}
