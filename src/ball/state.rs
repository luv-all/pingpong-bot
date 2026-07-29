//! 공 비행 상태 (슈터 parked / in-flight).

/// 공 비행 상태.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// 슈터 발사구에 고정 대기
    Parked,
    /// 비행 중
    InFlight,
}

impl std::fmt::Display for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(match self {
            Self::Parked => "parked",
            Self::InFlight => "in flight",
        });
    }
}
