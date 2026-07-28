//! clap 인자.

use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::{CamCliArgs, CameraId, DEFAULT_FOV_Y_DEG, MAX_REPROJ_RMSE_PX};

#[derive(Parser, Debug)]
#[command(
    name = "calib_table_pnp",
    about = "탁구대 랜드마크 8점 클릭 → solvePnP(IPPE) → Calibration JSON"
)]
pub struct Args {
    #[command(flatten)]
    pub cam: CamCliArgs,

    /// 동영상/이미지 파일 (라이브 대신)
    #[arg(long)]
    pub path: Option<PathBuf>,

    /// 출력 Calibration JSON (기본 calibration.json)
    #[arg(short = 'o', long)]
    pub output: Option<PathBuf>,

    /// 기존 Calibration에 이 카메라를 upsert
    #[arg(long)]
    pub merge: Option<PathBuf>,

    /// 수직 FOV [deg] → fx/fy 근사 (dist=[]). 기본=B0332 HFOV70°→VFOV≈47.3°
    #[arg(long, default_value_t = DEFAULT_FOV_Y_DEG)]
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

pub fn resolve_camera_id(args: &Args) -> Result<CameraId, String> {
    return args.cam.camera_id();
}

pub fn resolve_output(args: &Args) -> PathBuf {
    return args
        .output
        .clone()
        .unwrap_or_else(|| PathBuf::from("calibration.json"));
}
