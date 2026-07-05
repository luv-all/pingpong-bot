//! ChArUco 보드 촬영 → 코너 검출 → 카메라 내부/외부 파라미터 계산 (plan §3.4).
//!
//! 산출물: `Calibration` serde 파일 → 런타임 부팅 시 불변값으로 로드.

use clap::Parser;

#[derive(Parser)]
#[command(name = "calib_charuco", about = "ChArUco 카메라 보정 도구")]
struct Args {}

fn main() {
    let _args = Args::parse();
    todo!("ChArUco 카메라 보정 (plan.md §3.4)");
}
