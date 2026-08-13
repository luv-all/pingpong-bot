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

    /// 베이스 z 후보 [m] — **바닥(z=0) 기준 절대 좌표**.
    /// 실측 마운트는 0.815 (`defaults::rail_frame`); 스윕은 그 근방을 덮는다.
    #[arg(long, allow_hyphen_values = true, default_value_t = pingpong_bot::defaults::RAIL_MOUNT_Z_M - 0.075)]
    pub base_z_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = pingpong_bot::defaults::RAIL_MOUNT_Z_M + 0.075)]
    pub base_z_max: f64,
    #[arg(long, default_value_t = 8)]
    pub base_z_steps: usize,

    #[arg(long)]
    pub json: bool,

    /// 상위 몇 개 후보를 출력할지.
    #[arg(long, default_value_t = 5)]
    pub top_n: usize,
}
