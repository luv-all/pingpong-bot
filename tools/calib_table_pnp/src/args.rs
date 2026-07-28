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

    /// 출력 Calibration JSON. 생략 시 `cam{id}.json`
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

/// `-o` 없으면 `cam{id}.json` (카메라별 파일 관례).
pub fn resolve_output(args: &Args, cam_id: CameraId) -> PathBuf {
    return args
        .output
        .clone()
        .unwrap_or_else(|| PathBuf::from(format!("cam{}.json", cam_id.0)));
}

/// 공유 pending 번들 (`cameras[]` upsert). `-o`와 같은 디렉터리.
pub fn pending_path(args: &Args) -> PathBuf {
    const NAME: &str = "calibration.pending.json";
    if let Some(ref output) = args.output {
        if let Some(parent) = output.parent() {
            if !parent.as_os_str().is_empty() {
                return parent.join(NAME);
            }
        }
    }
    return PathBuf::from(NAME);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args_with_output(output: Option<&str>) -> Args {
        return Args {
            cam: CamCliArgs::parse_from(["x", "--cam", "left"]),
            path: None,
            output: output.map(PathBuf::from),
            merge: None,
            fov_y: DEFAULT_FOV_Y_DEG,
            max_rmse: MAX_REPROJ_RMSE_PX,
            from_pixels: None,
            validate: None,
        };
    }

    #[test]
    fn default_output_is_cam_id_json() {
        let args = args_with_output(None);
        assert_eq!(
            resolve_output(&args, CameraId(0)),
            PathBuf::from("cam0.json")
        );
        assert_eq!(pending_path(&args), PathBuf::from("calibration.pending.json"));
    }

    #[test]
    fn pending_is_shared_bundle_name() {
        let args = args_with_output(Some("cam1.json"));
        assert_eq!(pending_path(&args), PathBuf::from("calibration.pending.json"));
        let nested = args_with_output(Some("out/calibration.json"));
        assert_eq!(
            pending_path(&nested),
            PathBuf::from("out/calibration.pending.json")
        );
    }
}
