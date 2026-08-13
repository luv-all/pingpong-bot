//! CLI 인자.

use clap::Parser;

#[derive(Parser)]
#[command(
    name = "verify_mirror",
    about = "듀얼 MX-64 미러 페어(ID1↔ID2) 정렬 오차를 반복 측정하는 독립 진단 도구"
)]
pub struct Args {
    /// Dynamixel 포트 오버라이드 (`DynamixelConfig::default().port`보다 우선).
    #[arg(long)]
    pub dxl_port: Option<String>,
}
