//! 스테레오/멀티 선택용 CLI.

use crate::camera;
use clap::Parser;

use super::cam_cli_args::CamCliArgs;
use super::cam_stream_args::CamStreamArgs;
use crate::defaults::calib::DEFAULT_STEREO_CAM_ROLES;

/// 스테레오/멀티 선택용 (`left,right` 기본). `cam-preview`처럼 한 대만 열 수도 있는 툴.
#[derive(Parser, Debug, Clone)]
pub struct StereoCamCliArgs {
    /// 로봇 기준 역할. 예: `--cam left,right`
    #[arg(
        long = "cam",
        value_enum,
        value_delimiter = ',',
        default_values_t = DEFAULT_STEREO_CAM_ROLES
    )]
    pub cam: Vec<camera::Role>,

    #[command(flatten)]
    pub stream: CamStreamArgs,
}

impl StereoCamCliArgs {
    pub fn as_cam_cli(&self) -> CamCliArgs {
        return CamCliArgs {
            cam: self.cam.clone(),
            stream: self.stream.clone(),
        };
    }
}
