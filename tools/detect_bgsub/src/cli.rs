use anyhow::Result;
use clap::Parser;
use pingpong_bot::{DetectToolOptions, run_detect_tool};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(about = "공 검출 실험")]
pub struct DetectArgs {
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
    /// highgui 프리뷰 끄기
    #[arg(long)]
    pub no_preview: bool,
    /// waitKey ms (기본: 파일/이미지 33, 라이브 1)
    #[arg(long)]
    pub wait_ms: Option<i32>,
}

impl DetectArgs {
    pub fn to_options(&self) -> DetectToolOptions {
        return DetectToolOptions {
            images: self.images.clone(),
            device: self.device,
            path: self.path.clone(),
            output: self.output.clone(),
            max_frames: self.max_frames,
            preview: !self.no_preview,
            wait_ms: self.wait_ms,
        };
    }
}

pub fn run_detect(
    name: &str,
    args: &DetectArgs,
    detector: &mut dyn pingpong_bot::BallDetector,
) -> Result<()> {
    return run_detect_tool(name, &args.to_options(), detector);
}
