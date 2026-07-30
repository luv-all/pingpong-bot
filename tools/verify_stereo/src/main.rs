//! 스테레오 캘리브 검증 — left/right OpenCV + SimScene 공 창.

mod args;
mod msg;
mod run;
mod sim_child;

use anyhow::Result;
use clap::Parser;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();
    if args.sim_child {
        return sim_child::run_sim_child();
    }
    return run::run_opencv(&args);
}
