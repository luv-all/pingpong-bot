use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::ColorSpace;

#[derive(Parser, Debug)]
#[command(about = "공 픽셀 픽커 → YCrCb/HSV inRange 범위 (dry-run 출력만)")]
pub struct Args {
    #[arg(long)]
    pub images: Option<PathBuf>,
    /// 웹캠 인덱스 (미지정·images/path 없으면 0)
    #[arg(long)]
    pub device: Option<i32>,
    #[arg(long)]
    pub path: Option<PathBuf>,
    /// 시작 색공간 (마스크·띠 미리보기). `s`로 토글
    #[arg(long, value_enum, default_value_t = ColorSpace::Ycrcb)]
    pub space: ColorSpace,
    /// min/max에 더할 여유 (0..=32). 채널별 clamp 0..=255
    #[arg(long, default_value_t = 3)]
    pub margin: u8,
    #[arg(long, default_value_t = 0)]
    pub max_frames: usize,
    #[arg(long)]
    pub wait_ms: Option<i32>,
}
