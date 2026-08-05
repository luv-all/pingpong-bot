//! clap 인자.

use clap::Parser;
use pingpong_bot::camera::StereoCamCliArgs;

#[derive(Parser, Debug)]
#[command(name = "cam-preview")]
pub struct Args {
    #[command(flatten)]
    pub cam: StereoCamCliArgs,

    /// 공을 검출해 **생 삼각측량** 궤적을 그린다. 필터를 안 거친 값이다.
    ///
    /// `data/calibration.json` 과 `data/colormask.json` 이 있어야 한다.
    #[arg(long)]
    pub track: bool,

    /// 궤적에 남겨 둘 점 개수.
    #[arg(long, default_value_t = 120)]
    pub trail: usize,
}
