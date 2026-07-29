//! 한 물리 스텝 입력.

use crate::shooter;

/// 한 물리 스텝 입력 — `controls` 뮤텍스를 물리 연산 동안 잡지 않기 위함.
pub struct SimStepInput<'a> {
    /// 현재 슈터 설정
    pub shooter: &'a shooter::Settings,
    /// 이번 스텝에 발사
    pub shoot: bool,
    /// 이번 스텝에 주차
    pub park: bool,
}
