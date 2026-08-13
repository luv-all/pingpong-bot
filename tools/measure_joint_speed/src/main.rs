//! 관절 1축 실측 속도 진단 — 독립 실행형.
//!
//! 소프트웨어의 `arm.max_joint_speed`(모터 데이터시트 무부하 RPM 기반, 실기
//! Velocity Limit 레지스터와 무관)는 실기와 별개로 관리된다. Velocity Limit
//! 레지스터를 바꾼 뒤 실제 도달 각속도가 달라졌는지 확인하려면 이 도구로
//! 직접 재보고, `arm.max_joint_speed`와 비교해야 한다.

mod args;
mod run;

use anyhow::Result;
use clap::Parser;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();
    return run::run(&args);
}
