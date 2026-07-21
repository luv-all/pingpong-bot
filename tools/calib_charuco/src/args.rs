//! clap 인자 · 보드 스펙 · 출력 경로.

use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::{CharucoBoardSpec, DEFAULT_CONFIG_PATH, calibration_path_from_config};

#[derive(Parser, Debug)]
#[command(
    name = "calib_charuco",
    about = "ChArUco 인터랙티브 보정 — Space 스냅·코너 확인·s 저장·종료 시 JSON"
)]
pub struct Args {
    /// 웹캠 인덱스 (미지정 시 0으로 인터랙티브)
    #[arg(long)]
    pub device: Option<i32>,

    /// 동영상 파일로 같은 UX
    #[arg(long)]
    pub path: Option<PathBuf>,

    /// 선별 프레임 저장 디렉터리 (기본 calib_frames/cam{id})
    #[arg(long, value_name = "DIR")]
    pub images_dir: Option<PathBuf>,

    /// 출력 Calibration JSON. 생략 시 --config 의 calibration_path, 없으면 calibration.json
    #[arg(short = 'o', long)]
    pub output: Option<PathBuf>,

    /// 런타임 TOML (기본 출력 경로용 calibration_path)
    #[arg(long, value_name = "PATH", default_value = DEFAULT_CONFIG_PATH)]
    pub config: PathBuf,

    /// 종료 시 보정에 필요한 최소 저장 장수
    #[arg(long, default_value_t = 10)]
    pub min_frames: usize,

    #[arg(long, default_value_t = 0)]
    pub camera_id: u8,

    #[arg(long)]
    pub emit_sim: Option<u8>,

    #[arg(long)]
    pub validate: Option<PathBuf>,

    /// UI 없이 이미지 폴더만으로 보정
    #[arg(long)]
    pub from_images: Option<PathBuf>,

    #[arg(long, default_value_t = 5)]
    pub squares_x: i32,
    #[arg(long, default_value_t = 7)]
    pub squares_y: i32,
    #[arg(long, default_value_t = 0.04)]
    pub square_length: f32,
    #[arg(long, default_value_t = 0.02)]
    pub marker_length: f32,
}

pub fn board_spec(args: &Args) -> CharucoBoardSpec {
    return CharucoBoardSpec {
        squares_x: args.squares_x,
        squares_y: args.squares_y,
        square_length_m: args.square_length,
        marker_length_m: args.marker_length,
    };
}

pub fn resolve_output(args: &Args) -> PathBuf {
    if let Some(ref out) = args.output {
        return out.clone();
    }
    if let Ok(Some(path)) = calibration_path_from_config(&args.config) {
        return path;
    }
    return PathBuf::from("calibration.json");
}
