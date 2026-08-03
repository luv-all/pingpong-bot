//! CLI 인자.

use clap::Parser;

use super::mode_arg::ModeArg;

#[derive(Parser)]
#[command(name = "pingpong-bot", about = "협력 랠리 핑퐁 로봇 런타임")]
pub struct Args {
    /// sim | real
    #[arg(long, value_enum, default_value = "sim")]
    pub mode: ModeArg,
    /// Dynamixel 포트 오버라이드 (`DynamixelConfig::default().port`보다 우선).
    #[arg(long)]
    pub dxl_port: Option<String>,
    /// debug 로그 (샷별 계획·하드웨어 상세).
    #[arg(long)]
    pub debug: bool,
    /// real: 모터·레일을 실제로 움직이지 않고 전체 체인만 리허설.
    #[arg(long)]
    pub dry_run: bool,
    /// real: 좌/우 검출 오버레이 프리뷰 창. 끄려면 `--preview=false`
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    pub preview: bool,
    /// real: 시작 시 센터(ready) 자세로 이동. 끄려면 `--home=false`
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    pub home: bool,
    /// real: 라이브 캠 대신 녹화 클립을 재생한다 (`fly_02` 또는 디렉터리).
    ///
    /// `data/clips/{scene}_{nn}/`. 녹화 당시 fps로 페이싱해 라이브와 같은 타이밍으로 돈다.
    #[arg(long, value_name = "NAME|DIR")]
    pub clip: Option<std::path::PathBuf>,
    /// real: 관전용 sim 창 (테이블·로봇·예측 도달점). 끄려면 `--sim=false`
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    pub sim: bool,
    /// real: 종료 시 토크를 뺀다. 기본은 켠 채로 둬서 팔이 주저앉지 않게 한다.
    #[arg(long)]
    pub release_torque: bool,
    /// real: 공을 기다리는 최대 시간 [s].
    #[arg(long, default_value_t = 60.0)]
    pub timeout_secs: f64,
    /// 탁구대 로봇 쪽 끝선에서 레일까지의 양수 거리 [m] (sim/real 공통).
    #[arg(long, default_value_t = 0.10)]
    pub table_distance_m: f64,
    /// 바닥에서 레일 프로파일 하단까지의 높이 [m] (sim/real 공통).
    #[arg(long, default_value_t = 0.88)]
    pub rail_bottom_z_m: f64,
    /// 타격 후보 탐색 시작 Y [m]. 0.00은 로봇 쪽 탁구대 끝선.
    #[arg(long, default_value_t = 0.00)]
    pub hit_y_min_m: f64,
    /// 타격 후보 탐색 끝 Y [m].
    #[arg(long, default_value_t = 0.55)]
    pub hit_y_max_m: f64,
    /// 타격 후보 Y 간격 [m]. 전체 스윙 플래너에서는 레일+팔 IK로 다시 걸러진다.
    #[arg(long, default_value_t = 0.025)]
    pub hit_y_step_m: f64,
    /// sim 공 발사 중심 X [m]. 생략하면 기본 슈터 위치.
    #[arg(long)]
    pub ball_launch_x_m: Option<f64>,
    /// sim 공 발사 중심 Y [m]. 생략하면 기본 슈터 위치.
    #[arg(long)]
    pub ball_launch_y_m: Option<f64>,
    /// sim 공 발사 중심 Z [m]. 생략하면 기본 슈터 위치.
    #[arg(long)]
    pub ball_launch_z_m: Option<f64>,
}

impl Args {
    /// CLI의 현장 치수를 로봇 내부 월드 좌표로 변환한다.
    pub fn rail_frame(&self) -> pingpong_bot::robot::RailFrame {
        return pingpong_bot::robot::RailFrame::from_table_distance(
            self.table_distance_m,
            self.rail_bottom_z_m,
        );
    }

    /// sim/real이 공유할 타격 후보 탐색 창.
    pub fn intercept_window(&self) -> pingpong_bot::robot::motion::InterceptWindow {
        return pingpong_bot::robot::motion::InterceptWindow {
            y_min: self.hit_y_min_m,
            y_max: self.hit_y_max_m,
            sample_step: self.hit_y_step_m,
        };
    }
}
