//! ROI 추적 검출 실험 — 런타임 `RoiDetector`와 동일.

mod cli;

use anyhow::Result;
use clap::Parser;
use pingpong_bot::RoiDetector;

use cli::{DetectArgs, run_detect};

fn main() -> Result<()> {
    let args = DetectArgs::parse();
    let mut detector = RoiDetector::new();
    run_detect("roi", &args, &mut detector)
}
