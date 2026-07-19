//! Contour·원형도 검출 실험 — 런타임 `ContourDetector`와 동일.

mod cli;

use anyhow::Result;
use clap::Parser;
use pingpong_bot::ContourDetector;

use cli::{DetectArgs, run_detect};

fn main() -> Result<()> {
    let args = DetectArgs::parse();
    let mut detector = ContourDetector::new();
    run_detect("contour", &args, &mut detector)
}
