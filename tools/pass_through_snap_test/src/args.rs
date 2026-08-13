//! CLI 인자.

use clap::Parser;

#[derive(Parser)]
#[command(
    name = "pass_through_snap_test",
    about = "j0/j1/j2를 임팩트 지점 너머(overshoot)로 밀면서, j3는 접었다가(backswing) 임팩트 시각에 맞춰 한계까지 스냅하는 동작을 실기에서 격리 테스트한다"
)]
pub struct Args {
    /// Dynamixel 포트 오버라이드 (`DynamixelConfig::default().port`보다 우선).
    #[arg(long)]
    pub dxl_port: Option<String>,

    /// 목표 접촉점(라켓 중심이 임팩트 순간 있어야 할 위치) [m], 월드 프레임.
    #[arg(long, allow_hyphen_values = true)]
    pub target_x: f64,
    #[arg(long, allow_hyphen_values = true)]
    pub target_y: f64,
    #[arg(long, allow_hyphen_values = true)]
    pub target_z: f64,

    /// 목표 접촉점 너머로 IK 목표를 얼마나 더 밀어둘지 [m].
    #[arg(long, default_value_t = 0.05)]
    pub overshoot_m: f64,

    /// j0/j1/j2가 overshoot 목표에 도달하는 전체 시간 [s].
    #[arg(long)]
    pub total_duration_secs: f64,

    /// 라켓이 실제 목표 접촉점을 지나는(공을 맞히는) 추정 시각 [s] — `total_duration_secs`보다 작아야 한다.
    #[arg(long)]
    pub impact_time_secs: f64,

    /// 손목이 접히는 목표 각도 [deg] (절대각).
    #[arg(long, allow_hyphen_values = true)]
    pub wrist_cocked_deg: f64,

    /// 손목이 접힌 각도까지 도달하는 시간 [s].
    #[arg(long)]
    pub backswing_duration_secs: f64,

    /// j0/j2가 정지에서 관절 속도 상한까지 가속하는 데 쓰는 시간 [s].
    #[arg(long, default_value_t = 0.060)]
    pub ramp_secs: f64,

    /// 손목 스냅이 노리는 속도를 관절 속도 상한의 이 비율까지로 제한한다 [무차원].
    #[arg(long, default_value_t = 0.85)]
    pub snap_velocity_margin: f64,

    /// 스트리밍 주기 [Hz].
    #[arg(long, default_value_t = 200.0)]
    pub poll_hz: f64,
}
