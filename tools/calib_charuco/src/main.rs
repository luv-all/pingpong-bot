//! ChArUco 보드 보정.
//!
//! - 기본: [`interactive`] — Space/s/n/q 라이브 캡처
//! - 보조: [`cli`] — `--from-images` / `--emit-sim` / `--validate`

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

    if let Some(n) = args.emit_sim {
        return cli::emit_sim(n, &args);
    }

    if let Some(dir) = &args.from_images {
        return cli::from_images(dir, &args);
    }

    return interactive::run(&args);
}
