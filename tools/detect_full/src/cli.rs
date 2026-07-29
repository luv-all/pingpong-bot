use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::{CamCliArgs, MonoOfflineArgs};

#[derive(Parser, Debug)]
#[command(about = "Detector 본선 — adaptive ROI 튜닝 + 단계 패널")]
pub struct Args {
    #[command(flatten)]
    pub cam: CamCliArgs,

    #[arg(long)]
    pub images: Option<PathBuf>,

    #[command(flatten)]
    pub offline: MonoOfflineArgs,

    /// 시작 시 ROI off
    #[arg(long)]
    pub no_roi: bool,
    #[arg(short = 'o', long)]
    pub output: Option<PathBuf>,
    #[arg(long, default_value_t = 300)]
    pub max_frames: usize,
    #[arg(long)]
    pub no_preview: bool,
    #[arg(long)]
    pub wait_ms: Option<i32>,
}
