use clap::Parser;
use pingpong_bot::camera::{CamCliArgs, MonoOfflineArgs};
use pingpong_bot::defaults::STILL_HIT_RADIUS_PX;

#[derive(Parser, Debug)]
#[command(about = "클립 등분 덤프 → 클릭 라벨 → data/detect_stills/manifest.json")]
pub struct Args {
    #[command(flatten)]
    pub cam: CamCliArgs,

    #[command(flatten)]
    pub offline: MonoOfflineArgs,

    /// 뽑을 스틸 수 (타임라인 등분). 2~3장은 무공(`n`)으로 남길 것
    #[arg(long, default_value_t = 10)]
    pub count: usize,

    /// hit 판정 반경 [px] — manifest에 저장
    #[arg(long, default_value_t = STILL_HIT_RADIUS_PX)]
    pub hit_radius: f64,
}
