//! clap 인자.

use std::path::PathBuf;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "quintic 스윙 모양 제약 없이 순수 토크 한계로 임팩트 도달 시간을 측정한다")]
pub struct Args {
    /// TOML 시나리오 파일. 여기 값들을 아래 CLI 플래그가 덮어쓴다.
    #[arg(long)]
    pub scenario: Option<PathBuf>,

    /// 카탈로그 로봇 id (`competition` | `urdf-test` | `4-dof`).
    #[arg(long)]
    pub robot: Option<String>,

    /// 시작 레일 x [m]. 생략하면 레일 중앙(`default_x()`).
    #[arg(long)]
    pub start_rail_x: Option<f64>,

    #[arg(long, allow_hyphen_values = true)]
    pub impact_x: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    pub impact_y: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    pub impact_z: Option<f64>,

    #[arg(long, allow_hyphen_values = true)]
    pub incoming_vx: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    pub incoming_vy: Option<f64>,
    #[arg(long, allow_hyphen_values = true)]
    pub incoming_vz: Option<f64>,

    /// 참고용 — 실제 예측이라면 이 안에 들어와야 할 여유 시간 [s]. 결과 판정에는
    /// 안 쓰고, 리포트에서 achieved_time과 나란히 비교만 한다.
    #[arg(long)]
    pub time_budget_secs: Option<f64>,

    /// 적분 스텝 [s].
    #[arg(long, default_value_t = 0.001)]
    pub dt: f64,

    /// 수렴하지 않을 때 포기하는 최대 시뮬레이션 시간 [s].
    #[arg(long, default_value_t = 2.0)]
    pub max_time_secs: f64,

    /// 사람이 읽는 표 대신 JSON으로 출력.
    #[arg(long)]
    pub json: bool,
}
