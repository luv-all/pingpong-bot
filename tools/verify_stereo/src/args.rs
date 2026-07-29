//! clap — stereo cams · clip · sim child.

use clap::Parser;
use pingpong_bot::{StereoOfflineArgs, StereoPairCliArgs};

#[derive(Parser, Debug)]
#[command(
    name = "verify_stereo",
    about = "스테레오 월드 격자 + 공 삼각측량 + SimScene 공 창"
)]
pub struct Args {
    /// 항상 left+right (`--cam` 없음)
    #[command(flatten)]
    pub cam: StereoPairCliArgs,

    #[command(flatten)]
    pub offline: StereoOfflineArgs,

    /// SimScene 자식 창 (테이블+공). `--sim false`로 끔.
    #[arg(long = "sim", default_value_t = true, action = clap::ArgAction::Set)]
    pub sim: bool,

    /// 내부: sim 자식 모드 (부모가 spawn, stdin으로 XYZ).
    #[arg(long = "sim-child", hide = true)]
    pub sim_child: bool,
}
