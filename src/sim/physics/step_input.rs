//! 한 물리 스텝 입력.

use crate::robot::RailFrame;
use crate::robot::motion::InterceptWindow;
use crate::sim::launch;

/// 한 물리 스텝 입력 — `controls` 뮤텍스를 물리 연산 동안 잡지 않기 위함.
pub struct SimStepInput<'a> {
    /// 현재 슈터 설정
    pub shooter: &'a launch::Settings,
    /// 이번 스텝에 발사
    pub shoot: bool,
    /// 이번 스텝에 주차
    pub park: bool,
    /// 요청된 레일 마운트 설치 위치. 공이 주차 상태일 때만 반영된다
    /// ([`crate::sim::physics::SimWorld::apply_rail_frame`]).
    pub rail_frame: RailFrame,
    /// 시뮬 GUI에서 설정한 타격 후보 Y 창.
    pub intercept: InterceptWindow,
}
