//! 탁구대 6점 랜드마크 → solvePnP(IPPE) → Calibration JSON.
//!
//! - 기본: [`interactive`] — Space 스냅 · 클릭 · s 저장
//! - 보조: [`cli`] — `--from-pixels` / `--validate`

mod args;
mod cli;
mod interactive;

use anyhow::Result;
use clap::Parser;

use args::Args;

fn main() -> Result<()> {
    let args = Args::parse();

    if let Some(path) = args.validate {
        return cli::validate(&path);
    }

    if let Some(path) = &args.from_pixels {
        return cli::from_pixels(path, &args);
    }

    return interactive::run(&args);
}
