//! 백그라운드 진행 상태.

use super::{Report, TOTAL_SHOTS};

/// 백그라운드 진행 상태.
#[derive(Debug, Clone)]
pub struct Progress {
    pub done: usize,
    pub total: usize,
    pub report: Option<Report>,
    pub error: Option<String>,
}

impl Default for Progress {
    fn default() -> Self {
        return Self {
            done: 0,
            total: TOTAL_SHOTS,
            report: None,
            error: None,
        };
    }
}
