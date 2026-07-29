//! clap 인자.

use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::defaults::calibration_pending_path;
use pingpong_bot::{
    CamCliArgs, CameraId, DEFAULT_CALIBRATION_PATH, DEFAULT_FOV_Y_DEG, MAX_REPROJ_RMSE_PX,
};

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

    /// 출력 Calibration JSON. 생략 시 [`DEFAULT_CALIBRATION_PATH`] (upsert)
    #[arg(short = 'o', long, default_value = DEFAULT_CALIBRATION_PATH)]
    pub output: PathBuf,

    /// 기존 Calibration에 이 카메라를 upsert (미지정 시 `-o`와 동일 파일에서 읽기)
    #[arg(long)]
    pub merge: Option<PathBuf>,

    /// 수직 FOV [deg] → fx/fy 근사 (dist=[]). 기본=B0332 HFOV70°→VFOV≈47.3°
    #[arg(long, default_value_t = DEFAULT_FOV_Y_DEG)]
    pub fov_y: f64,

    /// 재투영 RMSE 한도 [px]
    #[arg(long, default_value_t = MAX_REPROJ_RMSE_PX)]
    pub max_rmse: f64,

    /// Review 캔버스 외곽 패딩 [px]. 프레임 밖 랜드마크 클릭용 (0=비활성)
    #[arg(long, default_value_t = 16)]
    pub pad: i32,

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

/// 본파일 경로 — clap 기본이 SSOT, `-o`로만 덮어씀.
pub fn resolve_output(args: &Args) -> PathBuf {
    return args.output.clone();
}

/// 공유 pending 번들 (`cameras[]` upsert). `-o`와 같은 디렉터리.
pub fn pending_path(args: &Args) -> PathBuf {
    return calibration_pending_path(Some(&args.output));
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::defaults::calibration_path;

    fn args_with_output(output: &str) -> Args {
        return Args {
            cam: CamCliArgs::parse_from(["x", "--cam", "left"]),
            path: None,
            output: PathBuf::from(output),
            merge: None,
            fov_y: DEFAULT_FOV_Y_DEG,
            max_rmse: MAX_REPROJ_RMSE_PX,
            pad: 16,
            from_pixels: None,
            validate: None,
        };
    }

    #[test]
    fn default_output_is_calibration_ssot() {
        let args = Args::parse_from(["x", "--cam", "left"]);
        assert_eq!(resolve_output(&args), calibration_path());
        assert_eq!(
            pending_path(&args),
            PathBuf::from("data/calibration.pending.json")
        );
    }

    #[test]
    fn pending_follows_output_dir() {
        let nested = args_with_output("out/calibration.json");
        assert_eq!(
            pending_path(&nested),
            PathBuf::from("out/calibration.pending.json")
        );
    }
}
