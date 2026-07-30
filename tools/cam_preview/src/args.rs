//! clap 인자.

use clap::Parser;
use pingpong_bot::camera::StereoCamCliArgs;

#[derive(Parser, Debug)]
#[command(name = "cam-preview")]
pub struct Args {
    #[command(flatten)]
    pub cam: StereoCamCliArgs,
}
