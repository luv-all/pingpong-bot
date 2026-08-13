//! 듀얼 MX-64 미러 페어(ID1↔ID2) 정렬 오차 진단 — 독립 실행형.

mod args;
mod run;

use anyhow::Result;
use clap::Parser;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();
    return run::run(&args);
}
