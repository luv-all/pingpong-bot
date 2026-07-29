//! clap 인자.

use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::camera::{StereoOfflineArgs, StereoPairCliArgs};

#[derive(Parser, Debug)]
#[command(
    name = "measure_restitution",
    about = "반발계수 e 측정 → PhysicsParams::default() 스니펫. 영상 멀티캠 또는 수동 숫자"
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
    /// 파일 재생 타임라인 FPS (미지정 시 clip meta / 파일 메타)
    #[arg(long)]
    pub timeline_fps: Option<f64>,
    #[arg(long, value_name = "H0,H1,...")]
    pub heights: Option<String>,
    #[arg(long, value_name = "VIN:VOUT,...")]
    pub vz_pairs: Option<String>,
    #[arg(long)]
    pub sim: bool,
    #[arg(long)]
    pub sim_ballistics: bool,
    #[arg(long, value_name = "PATH")]
    pub drag_csv: Option<PathBuf>,
    #[arg(long, default_value_t = 0.40)]
    pub drop_height: f64,
}
