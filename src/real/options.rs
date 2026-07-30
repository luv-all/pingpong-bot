//! 실기 단발 실행 옵션 (CLI에서 온 것만).
//!
//! 숫자·배선 SSOT는 `pingpong_bot::defaults`다. 여기엔 모드를 바꾸는 플래그만 둔다.

use crate::cli::Args;

/// `--mode real` 실행 옵션.
#[derive(Debug, Clone)]
pub struct Options {
    /// 모터·레일을 실제로 움직이지 않고 전체 체인만 리허설.
    pub dry_run: bool,
    /// 좌/우 검출 오버레이 프리뷰 창.
    pub preview: bool,
    /// 시작 시 센터(ready) 자세로 이동.
    pub home: bool,
    /// 관전용 sim 창 (자식 프로세스).
    pub sim: bool,
    /// 종료 시 토크를 빼서 팔을 손으로 옮길 수 있게 한다.
    pub release_torque: bool,
    /// 공을 기다리는 최대 시간 [s].
    pub timeout_secs: f64,
    /// Dynamixel 포트 오버라이드.
    pub dxl_port: Option<String>,
}

impl Options {
    pub fn from_args(args: &Args) -> Self {
        return Self {
            dry_run: args.dry_run,
            preview: args.preview,
            home: args.home,
            sim: args.sim,
            release_torque: args.release_torque,
            timeout_secs: args.timeout_secs,
            dxl_port: args.dxl_port.clone(),
        };
    }
}
