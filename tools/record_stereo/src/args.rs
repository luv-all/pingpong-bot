//! clap 인자.

use std::path::PathBuf;

use clap::Parser;
use pingpong_bot::camera::StereoPairCliArgs;

use crate::scene::Scene;

#[derive(Parser, Debug)]
#[command(name = "record-stereo")]
pub struct Args {
    #[command(flatten)]
    pub cam: StereoPairCliArgs,

    /// 장면 태그 (클립 디렉터리 prefix). 실행 중 변경 없음.
    #[arg(long, value_enum, default_value_t = Scene::Fly)]
    pub scene: Scene,

    /// 클립 루트 (`data/clips/{scene}_{nn}/`)
    #[arg(long, default_value = "data/clips")]
    pub out: PathBuf,

    /// Space 기준 과거 보관 초
    #[arg(long, default_value_t = 10.0)]
    pub preroll: f64,

    /// Space 이후 추가 녹화 초
    #[arg(long, default_value_t = 2.0)]
    pub postroll: f64,
}
