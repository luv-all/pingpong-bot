//! clap 인자.

use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::MAX_REPROJ_RMSE_PX;

#[derive(Parser, Debug)]
#[command(
    name = "calib_table_pnp",
    about = "탁구대 랜드마크 8점 클릭 → solvePnP(IPPE) → Calibration JSON"
)]
pub struct Args {
    /// 웹캠 인덱스 (기본 인터랙티브)
    #[arg(long)]
    pub device: Option<i32>,

    /// 동영상/이미지 파일
    #[arg(long)]
    pub path: Option<PathBuf>,

    /// 출력 Calibration JSON (기본 calibration.json)
    #[arg(short = 'o', long)]
    pub output: Option<PathBuf>,

    /// 기존 Calibration에 이 카메라를 upsert
    #[arg(long)]
    pub merge: Option<PathBuf>,

    #[arg(long, default_value_t = 0)]
    pub camera_id: u8,

    /// 수직 FOV [deg] → fx/fy 근사 (dist=[])
    #[arg(long, default_value_t = 55.0)]
    pub fov_y: f64,

    /// 재투영 RMSE 한도 [px]
    #[arg(long, default_value_t = MAX_REPROJ_RMSE_PX)]
    pub max_rmse: f64,

    /// 픽셀 JSON으로 PnP만 (인터랙티브 없음). 예: {"width":640,"height":480,"pixels":[[u,v],...]}
    #[arg(long)]
    pub from_pixels: Option<PathBuf>,

    /// JSON 로드 검증만
    #[arg(long)]
    pub validate: Option<PathBuf>,
}

pub fn resolve_output(args: &Args) -> PathBuf {
    return args
        .output
        .clone()
        .unwrap_or_else(|| PathBuf::from("calibration.json"));
}
