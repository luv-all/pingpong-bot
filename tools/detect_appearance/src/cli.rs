use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::CamCliArgs;

#[derive(Parser, Debug)]
#[command(about = "appearance 좌우 비교 — colormask | contour")]
pub struct Args {
    #[command(flatten)]
    pub cam: CamCliArgs,

    #[arg(long)]
    pub images: Option<PathBuf>,
    #[arg(long)]
    pub path: Option<PathBuf>,
    #[arg(short = 'o', long)]
    pub output: Option<PathBuf>,
    #[arg(long, default_value_t = 300)]
    pub max_frames: usize,
    #[arg(long)]
    pub no_preview: bool,
    #[arg(long)]
    pub wait_ms: Option<i32>,
}
