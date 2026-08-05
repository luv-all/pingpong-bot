use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::camera::{CamCliArgs, MonoOfflineArgs};

#[derive(Parser, Debug)]
#[command(about = "vision 검출기 단계 패널 — 레이어마다 마스크 하나")]
pub struct Args {
    #[command(flatten)]
    pub cam: CamCliArgs,

    #[command(flatten)]
    pub offline: MonoOfflineArgs,

    #[arg(short = 'o', long)]
    pub output: Option<PathBuf>,
    #[arg(long, default_value_t = 2000)]
    pub max_frames: usize,
    #[arg(long)]
    pub no_preview: bool,
    #[arg(long)]
    pub wait_ms: Option<i32>,
    /// 패널 축소 배율.
    #[arg(long, default_value_t = 0.5)]
    pub scale: f64,
}
