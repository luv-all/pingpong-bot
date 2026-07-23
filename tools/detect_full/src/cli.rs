use std::path::PathBuf;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "fuse 본선 — adaptive ROI 튜닝 + 단계 패널")]
pub struct Args {
    #[arg(long)]
    pub images: Option<PathBuf>,
    #[arg(long)]
    pub device: Option<i32>,
    #[arg(long)]
    pub path: Option<PathBuf>,
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
