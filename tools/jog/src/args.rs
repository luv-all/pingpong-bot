//! clap 인자.

use std::path::PathBuf;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    name = "jog",
    about = "관절·레일 인터랙티브 조그 GUI (sim 미리보기 + Apply)"
)]
pub struct Args {
    /// Dynamixel 시리얼 포트 (`DynamixelConfig::default().port` 덮어씀).
    #[arg(long)]
    pub port: Option<String>,
    /// AXL.dll 경로 (`RailConfig::default().dll_path` 덮어씀).
    #[arg(long)]
    pub dll_path: Option<PathBuf>,
    /// 시리얼·DLL 없이 변환·IK·executor만.
    #[arg(long)]
    pub dry_run: bool,
    /// debug 로그 (통신 재시도·AXL 실패 code 등).
    #[arg(long)]
    pub debug: bool,
}
