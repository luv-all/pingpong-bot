//! CLI 인자.

use clap::Parser;

#[derive(Parser)]
#[command(
    name = "measure_joint_speed",
    about = "관절 하나를 왕복 스윕해 실제 도달 각속도를 실측 — 소프트웨어 상한(max_joint_speed)과 비교용"
)]
pub struct Args {
    /// Dynamixel 포트 오버라이드 (`DynamixelConfig::default().port`보다 우선).
    #[arg(long)]
    pub dxl_port: Option<String>,

    /// 관절 인덱스 (0=j0 요, 1=j1 어깨, 2=j2 팔꿈치, 3=j3 손목).
    #[arg(long)]
    pub joint: usize,

    /// 현재 위치에서 이만큼 더한 목표로 한 번 이동한다 [deg].
    #[arg(long, default_value_t = 20.0)]
    pub amplitude_deg: f64,

    /// 위치 폴링 주기 [Hz].
    #[arg(long, default_value_t = 200.0)]
    pub poll_hz: f64,

    /// 안전 상한 — 이 시간이 지나면 정지 여부와 무관하게 폴링을 끝낸다 [s].
    #[arg(long, default_value_t = 2.0)]
    pub timeout_secs: f64,
}
