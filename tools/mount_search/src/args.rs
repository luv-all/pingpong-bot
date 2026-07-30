//! clap 인자.

use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "레일 마운트 위치(테이블과의 거리·높이) 스윕으로 최적 위치를 찾는다")]
pub struct Args {
    /// 테이블과의 거리(y) 후보 최소값 [m] - `BASE_Y` 관례 좌표계.
    #[arg(long, allow_hyphen_values = true, default_value_t = -0.05)]
    pub base_y_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.10)]
    pub base_y_max: f64,
    #[arg(long, default_value_t = 7)]
    pub base_y_steps: usize,

    /// 테이블 면 대비 높이 오프셋 후보 [m] - 실기는 약 +0.03.
    #[arg(long, allow_hyphen_values = true, default_value_t = -0.02)]
    pub height_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.08)]
    pub height_max: f64,
    #[arg(long, default_value_t = 6)]
    pub height_steps: usize,

    #[arg(long)]
    pub json: bool,

    /// 상위 몇 개 후보를 출력할지.
    #[arg(long, default_value_t = 5)]
    pub top_n: usize,
}
