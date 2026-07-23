use std::path::PathBuf;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "appearance 좌우 비교 — colormask | contour")]
pub struct Args {
    #[arg(long)]
    pub images: Option<PathBuf>,
    #[arg(long)]
    pub device: Option<i32>,
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
