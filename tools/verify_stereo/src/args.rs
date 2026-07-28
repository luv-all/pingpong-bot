//! clap — calibration SSOT · stereo cams · sim child.

use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::{DEFAULT_CALIBRATION_PATH, StereoPairCliArgs};

#[derive(Parser, Debug)]
#[command(
    name = "verify_stereo",
    about = "스테레오 월드 격자 + 공 삼각측량 + SimScene 공 창"
)]
pub struct Args {
    /// 항상 left+right (`--cam` 없음)
    #[command(flatten)]
    pub cam: StereoPairCliArgs,

    /// Calibration JSON. 생략 시 [`DEFAULT_CALIBRATION_PATH`]
    #[arg(long, default_value = DEFAULT_CALIBRATION_PATH)]
    pub calibration: PathBuf,

    /// 녹화 파일 (left,right 순서). 없으면 라이브.
    #[arg(long = "video", value_name = "PATH")]
    pub videos: Vec<PathBuf>,

    /// SimScene 자식 창 (테이블+공). `--sim false`로 끔.
    #[arg(long = "sim", default_value_t = true, action = clap::ArgAction::Set)]
    pub sim: bool,

    /// 내부: sim 자식 모드 (부모가 spawn, stdin으로 XYZ).
    #[arg(long = "sim-child", hide = true)]
    pub sim_child: bool,
}
