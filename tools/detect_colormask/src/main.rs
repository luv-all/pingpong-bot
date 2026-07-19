//! 색상 마스크로 공 검출 실험 — 런타임 `ColormaskDetector`와 동일.

mod cli;

use anyhow::Result;
use clap::Parser;
use pingpong_bot::{ColormaskConfig, ColormaskDetector};

use cli::{DetectArgs, run_detect};

fn main() -> Result<()> {
    let args = DetectArgs::parse();
    let mut detector = ColormaskDetector::new(ColormaskConfig::default());
    run_detect("colormask", &args, &mut detector)
}
