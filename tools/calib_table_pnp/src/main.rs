//! 탁구대 8점 랜드마크 → solvePnP(IPPE) → Calibration JSON.
//!
//! - 기본: [`interactive`] — Space 스냅 · 클릭 · 점 미세조정 · s 저장
//! - 조정: [`adjust`] — 선택/이동/undo/bounded refine (순수 로직)
//! - 그리기: [`overlay`] — 패딩 캔버스 · 클릭 · 재투영 · 잔차
//! - 보조: [`cli`] — `--from-pixels` / `--validate`

mod adjust;
mod args;
mod cli;
mod interactive;
mod overlay;

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
