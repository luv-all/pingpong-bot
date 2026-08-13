//! j0/j1/j2 overshoot + j3 backswing-스냅 격리 테스트 — 독립 실행형.

mod args;
mod geometry;
mod plan;
mod run;
mod wrist_motion;

use anyhow::Result;
use clap::Parser;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();
    return run::run(&args);
}
