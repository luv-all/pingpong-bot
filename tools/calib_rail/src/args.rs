//! clap — 레일 홈잉 대상 엔드·DLL 경로.

use std::path::PathBuf;

use clap::{Parser, ValueEnum};

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum RailEndArg {
    Min,
    Max,
}

impl From<RailEndArg> for pingpong_bot::hardware::rail::RailEnd {
    fn from(end: RailEndArg) -> Self {
        return match end {
            RailEndArg::Min => pingpong_bot::hardware::rail::RailEnd::Min,
            RailEndArg::Max => pingpong_bot::hardware::rail::RailEnd::Max,
        };
    }
}

#[derive(Parser, Debug)]
#[command(
    name = "calib_rail",
    about = "레일을 물리적 엔드스톱까지 저속 이동해 영점을 다시 잡고 data/rail_calibration.json에 저장한다"
)]
pub struct Args {
    /// 홈잉이 향할 엔드스톱 방향.
    #[arg(long, value_enum, default_value = "min")]
    pub end: RailEndArg,
    /// AXL.dll 경로 (`RailConfig::default().dll_path` 덮어씀).
    #[arg(long)]
    pub dll_path: Option<PathBuf>,
    /// debug 로그 (AXL API 실패 code 등).
    #[arg(long)]
    pub debug: bool,
}
