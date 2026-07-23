//! sim GUI·물리 스레드가 공유하는 런타임 제어.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use super::ball_script::BallScript;
use super::shooter::BallShooterSettings;

/// GUI에서 바꾸고 물리 스레드가 읽는 sim 런타임 상태.
#[derive(Debug)]
pub struct SimRuntimeControls {
    /// 발사 파라미터 (GUI 슬라이더)
    pub shooter: BallShooterSettings,
    /// sim 시간 배율 (1.0 = 실시간)
    pub time_scale: f64,
    /// true면 commit 시 quintic 대신 순수 토크 bang-bang을 계획한다 - GUI
    /// "Bang-bang swing (debug)" 체크박스가 매 프레임 반영한다.
    pub use_bang_bang_swing: bool,
    /// 발사 버튼 — 물리 스레드가 소비
    pub shoot_requested: bool,
    /// 공 회수 — 슈터에 다시 주차
    pub park_requested: bool,
    /// 물리 스레드로 넘길 공 동역학 이벤트 큐
    pub ball_script_queue: BallScript,
}

impl Default for SimRuntimeControls {
    fn default() -> Self {
        return Self {
            shooter: BallShooterSettings::default(),
            time_scale: 1.0,
            use_bang_bang_swing: false,
            shoot_requested: false,
            park_requested: false,
            ball_script_queue: BallScript::new(),
        };
    }
}

impl SimRuntimeControls {
    /// GUI 발사 버튼.
    pub fn request_shoot(&mut self) {
        self.shoot_requested = true;
    }

    /// GUI 공 회수 버튼.
    pub fn request_park(&mut self) {
        self.park_requested = true;
    }

    /// 공 동역학 이벤트를 큐에 넣는다 (물리 스레드가 다음 스텝에 소비).
    pub fn enqueue_ball_script(&mut self, script: BallScript) {
        self.ball_script_queue.extend(script);
    }
}

/// 파이프라인·GUI 종료 신호.
pub fn new_shutdown_flag() -> Arc<AtomicBool> {
    return Arc::new(AtomicBool::new(false));
}
