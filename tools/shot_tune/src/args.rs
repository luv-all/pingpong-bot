//! clap 인자.

use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    about = "슈터 발사 기하(speed × pitch × height_offset)를 실제 Rapier 랠리 성공률로 스윕한다"
)]
pub struct Args {
    /// 로봇 프리셋 id (`4-dof` | `primitive` | `urdf-test`).
    #[arg(long, default_value = "4-dof")]
    pub robot: String,

    #[arg(long, default_value_t = 5.0)]
    pub speed_min: f64,
    #[arg(long, default_value_t = 11.0)]
    pub speed_max: f64,
    #[arg(long, default_value_t = 7)]
    pub speed_steps: usize,

    #[arg(long, allow_hyphen_values = true, default_value_t = -12.0)]
    pub pitch_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 4.0)]
    pub pitch_max: f64,
    #[arg(long, default_value_t = 9)]
    pub pitch_steps: usize,

    #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
    pub height_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.30)]
    pub height_max: f64,
    #[arg(long, default_value_t = 7)]
    pub height_steps: usize,

    #[arg(long, allow_hyphen_values = true, default_value_t = -0.10)]
    pub base_y_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = -0.10)]
    pub base_y_max: f64,
    #[arg(long, default_value_t = 1)]
    pub base_y_steps: usize,

    #[arg(long, allow_hyphen_values = true, default_value_t = 0.05)]
    pub mount_height_min: f64,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.05)]
    pub mount_height_max: f64,
    #[arg(long, default_value_t = 1)]
    pub mount_height_steps: usize,

    #[arg(long, default_value_t = 12)]
    pub shots: usize,

    #[arg(long)]
    pub use_random_speed: bool,

    #[arg(long, default_value_t = 20260723)]
    pub seed: u64,

    #[arg(long)]
    pub start_from_table_center: bool,

    #[arg(long)]
    pub require_legal: bool,

    #[arg(long)]
    pub sort_by_legal: bool,

    #[arg(long)]
    pub rest_pose_search: bool,

    #[arg(long)]
    pub explain: bool,

    #[arg(long)]
    pub json: bool,

    #[arg(long, default_value_t = 12)]
    pub top_n: usize,
}
