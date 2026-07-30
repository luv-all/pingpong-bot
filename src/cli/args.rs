//! CLI 인자.

use clap::Parser;

use super::mode_arg::ModeArg;

#[derive(Parser)]
#[command(name = "pingpong-bot", about = "협력 랠리 핑퐁 로봇 런타임")]
pub struct Args {
    /// sim | real
    #[arg(long, value_enum, default_value = "sim")]
    pub mode: ModeArg,
    /// Dynamixel 포트 오버라이드 (`DynamixelConfig::default().port`보다 우선).
    #[arg(long)]
    pub dxl_port: Option<String>,
    /// debug 로그 (샷별 계획·하드웨어 상세).
    #[arg(long)]
    pub debug: bool,
    /// real: 모터·레일을 실제로 움직이지 않고 전체 체인만 리허설.
    #[arg(long)]
    pub dry_run: bool,
    /// real: 좌/우 검출 오버레이 프리뷰 창. 끄려면 `--preview=false`
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    pub preview: bool,
    /// real: 시작 시 센터(ready) 자세로 이동. 끄려면 `--home=false`
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    pub home: bool,
    /// real: 공을 기다리는 최대 시간 [s].
    #[arg(long, default_value_t = 60.0)]
    pub timeout_secs: f64,
}
