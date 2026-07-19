//! 배경 차분 검출 실험 — 런타임 `BgSubDetector`와 동일.

mod cli;

use anyhow::Result;
use clap::Parser;
use pingpong_bot::BgSubDetector;

use cli::{DetectArgs, run_detect};

fn main() -> Result<()> {
    let args = DetectArgs::parse();
    let mut detector = BgSubDetector::new();
    run_detect("bgsub", &args, &mut detector)
}
