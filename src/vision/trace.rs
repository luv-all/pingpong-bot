//! 툴이 보는 단계별 산출물. 본선([`Vision::feed`](super::Vision::feed))은 안 만든다.

use super::detect::{Candidate, Mask};
use super::fit::Outcome;

/// 한 프레임의 진단.
#[derive(Default)]
pub struct Trace {
    /// `(이름, 그 단계 직후 마스크)`. 툴이 단계를 하드코딩하지 않는다.
    pub stages: Vec<(&'static str, Mask)>,
    /// 후보와 편차. 남는 개수가 마할라노비스 연관이 필요한지의 근거가 된다.
    pub candidates: Vec<(Candidate, f64)>,
    pub chosen: Option<Candidate>,
    pub outcome: Option<Outcome>,
}
