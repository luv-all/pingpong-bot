use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::{CamCliArgs, ColorSpace, MonoOfflineArgs};

#[derive(Parser, Debug)]
#[command(about = "공 픽셀 픽커 → YCrCb/HSV inRange → data/colormask.json upsert")]
pub struct Args {
    #[command(flatten)]
    pub cam: CamCliArgs,

    #[arg(long)]
    pub images: Option<PathBuf>,

    #[command(flatten)]
    pub offline: MonoOfflineArgs,

    /// 시작 색공간 (마스크·띠 미리보기). `s`로 토글
    #[arg(long, value_enum, default_value_t = ColorSpace::Ycrcb)]
    pub space: ColorSpace,
    /// 퍼센타일 구간에 더할 여유 (0..=32). 채널별 clamp 0..=255
    #[arg(long, default_value_t = 3)]
    pub margin: u8,
    /// 채널별 양꼬리 절단 % (0=min/max, 10 → p10..p90). 하이라이트·혼색 아웃라이어 억제
    #[arg(long, default_value_t = 10.0)]
    pub trim: f64,
    #[arg(long, default_value_t = 0)]
    pub max_frames: usize,
    #[arg(long)]
    pub wait_ms: Option<i32>,
}
