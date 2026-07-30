//! clap 인자.

use clap::Parser;
use pingpong_bot::camera::{StereoOfflineArgs, StereoPairCliArgs};

#[derive(Parser, Debug)]
#[command(
    name = "measure_friction",
    about = "테이블 마찰 μ 측정 → PhysicsParams::default() 스니펫. 영상 멀티캠 또는 수동 숫자"
)]
pub struct Args {
    #[command(flatten)]
    pub offline: StereoOfflineArgs,
    #[command(flatten)]
    pub cam: StereoPairCliArgs,
    #[arg(long)]
    pub no_preview: bool,
    #[arg(long, default_value_t = 33)]
    pub wait_ms: i32,
    #[arg(long, default_value_t = 10_000)]
    pub max_frames: usize,
    /// 파일 재생 타임라인 FPS
    #[arg(long)]
    pub timeline_fps: Option<f64>,
    #[arg(long, value_name = "VIN:VOUT,...")]
    pub vt_pairs: Option<String>,
    #[arg(long)]
    pub sim: bool,
    #[arg(long, default_value_t = 2.0)]
    pub horiz_speed: f64,
    #[arg(long, default_value_t = 0.25)]
    pub drop_height: f64,
}
