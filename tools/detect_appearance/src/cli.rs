use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::DEFAULT_CONFIG_PATH;

#[derive(Parser, Debug)]
#[command(about = "appearance 좌우 비교 — colormask | contour")]
pub struct Args {
    /// 런타임 TOML (`[vision]` SSOT)
    #[arg(long, value_name = "PATH", default_value = DEFAULT_CONFIG_PATH)]
    pub config: PathBuf,
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
