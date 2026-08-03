//! sim GUI·물리 스레드가 공유하는 런타임 제어.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use crate::robot::RailFrame;
use crate::robot::motion::InterceptWindow;
use crate::sim::launch;

/// GUI에서 바꾸고 물리 스레드가 읽는 sim 런타임 상태.
#[derive(Debug)]
pub struct SimRuntimeControls {
    /// 발사 파라미터 (GUI 슬라이더)
    pub shooter: launch::Settings,
    /// 레일 철제 프로파일 설치 위치 (GUI "Rig" 슬라이더).
    ///
    /// 실물에서도 조정 가능한 축만 담는다 — 프로파일 두께는
    /// [`RAIL_THICKNESS`](crate::constants::geometry::RAIL_THICKNESS) 고정.
    /// 월드는 **공이 주차된 동안만** 이 값을 팔에 반영한다
    /// (`SimWorld::apply_rail_frame`) — 비행 중 베이스가 움직이면 이미 계획된
    /// 궤적이 옛 베이스를 기준으로 남는다.
    pub rail_frame: RailFrame,
    /// 예측 공 궤적에서 IK를 시도할 타격 Y 범위.
    /// 공이 주차된 동안만 시뮬 월드에 반영한다.
    pub intercept: InterceptWindow,
    /// sim 시간 배율 (1.0 = 실시간)
    pub time_scale: f64,
    /// true면 commit 시 quintic 대신 순수 토크 bang-bang을 계획한다 - GUI
    /// "Bang-bang swing (debug)" 체크박스가 매 프레임 반영한다.
    pub use_bang_bang_swing: bool,
    /// true면 commit 시 quintic 대신 IK 없는 고정 스윙 딕셔너리
    /// (`robot::motion::fixed_swing`)로 계획한다 - GUI 체크박스가 매 프레임 반영한다.
    pub use_fixed_swing_dictionary: bool,
    /// 발사 버튼 — 물리 스레드가 소비
    pub shoot_requested: bool,
    /// 공 회수 — 슈터에 다시 주차
    pub park_requested: bool,
}

impl Default for SimRuntimeControls {
    fn default() -> Self {
        return Self {
            shooter: launch::Settings::default(),
            rail_frame: crate::defaults::rail_frame(),
            intercept: InterceptWindow::default(),
            time_scale: 1.0,
            use_bang_bang_swing: false,
            use_fixed_swing_dictionary: false,
            shoot_requested: false,
            park_requested: false,
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

    /// 파이프라인·GUI 종료 신호.
    pub fn new_shutdown() -> Arc<AtomicBool> {
        return Arc::new(AtomicBool::new(false));
    }
}
