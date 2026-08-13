//! clap — RailQueue 실기 검증 시나리오 파라미터.

use std::path::PathBuf;

use clap::{Parser, ValueEnum};

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum Scenario {
    /// 1차 이동 시작 직후(진행률 낮음)에 2차 명령을 보낸다.
    Early,
    /// 1차 이동이 거의 끝났을 때(진행률 높음)에 2차 명령을 보낸다.
    Late,
    /// Early와 Late를 순서대로 모두 실행한다.
    Both,
}

#[derive(Parser, Debug)]
#[command(
    name = "rail_queue_bench",
    about = "RailQueue가 실기에서 '이전 이동을 항상 끝까지 마친 뒤에만 다음 명령을 보낸다'를 지키는지 검증한다"
)]
pub struct Args {
    /// AXL.dll 경로 (`RailConfig::default().dll_path` 덮어씀).
    #[arg(long)]
    pub dll_path: Option<PathBuf>,
    /// 실행할 시나리오.
    #[arg(long, value_enum, default_value = "both")]
    pub scenario: Scenario,
    /// 1차 이동(중앙 → 최좌단) 소요 시간 [s].
    #[arg(long, default_value_t = 3.0)]
    pub duration1_secs: f64,
    /// 2차 이동(최좌단↔중앙 사이 목표) 소요 시간 [s].
    #[arg(long, default_value_t = 1.5)]
    pub duration2_secs: f64,
    /// 2차 목표 위치 — 최좌단(0.0)과 중앙(1.0) 사이 비율.
    #[arg(long, default_value_t = 0.5)]
    pub target2_fraction: f64,
    /// 시나리오 시작 전 준비 이동(현재 위치 → 중앙) 소요 시간 [s].
    ///
    /// `AxlRail::move_abs_m`(기본 최고속 7.5 m/s, 가감속 24 m/s²)로 블로킹
    /// 이동하면 시나리오 사이에 레일이 갑자기 전속력으로 튀는 느낌을 준다 —
    /// 실제 검증 이동과 같은 duration 기반 속도로 준비 이동한다.
    #[arg(long, default_value_t = 2.0)]
    pub prep_duration_secs: f64,
    /// Early 시나리오에서 2차 명령을 보내기까지, 1차 이동 duration 대비 비율.
    #[arg(long, default_value_t = 0.2)]
    pub early_fraction: f64,
    /// Late 시나리오에서 2차 명령을 보내기까지, 1차 이동 duration 대비 비율.
    #[arg(long, default_value_t = 0.8)]
    pub late_fraction: f64,
    /// debug 로그 (AXL API 실패 code 등).
    #[arg(long)]
    pub debug: bool,
}
