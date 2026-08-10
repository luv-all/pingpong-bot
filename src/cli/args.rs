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
    /// real: 라이브 캠 대신 녹화 클립을 재생한다 (`fly_02` 또는 디렉터리).
    ///
    /// `data/clips/{scene}_{nn}/`. 녹화 당시 fps로 페이싱해 라이브와 같은 타이밍으로 돈다.
    #[arg(long, value_name = "NAME|DIR")]
    pub clip: Option<std::path::PathBuf>,
    /// real: 종료 시 토크를 뺀다. 기본은 켠 채로 둬서 팔이 주저앉지 않게 한다.
    #[arg(long)]
    pub release_torque: bool,
    /// real: 공을 기다리는 최대 시간 [s].
    #[arg(long, default_value_t = 60.0)]
    pub timeout_secs: f64,
}
