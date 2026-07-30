//! 양쪽 캠 필수 툴용 CLI.

use clap::Parser;

use super::cam_cli_args::CamCliArgs;
use super::cam_stream_args::CamStreamArgs;
use crate::defaults::calib::DEFAULT_STEREO_CAM_ROLES;

/// 양쪽 캠 **필수** 툴용 — `--cam` 없음. 항상 left+right.
#[derive(Parser, Debug, Clone)]
pub struct StereoPairCliArgs {
    #[command(flatten)]
    pub stream: CamStreamArgs,
}

impl StereoPairCliArgs {
    pub fn as_cam_cli(&self) -> CamCliArgs {
        return CamCliArgs {
            cam: DEFAULT_STEREO_CAM_ROLES.to_vec(),
            stream: self.stream.clone(),
        };
    }
}
