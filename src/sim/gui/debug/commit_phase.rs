//! 스윙 commit 게이트 단계.

/// 스윙 commit 게이트 단계.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CommitPhase {
    #[default]
    Idle,
    WaitMidcourt,
    WaitWindow,
    InWindow,
    TooLate,
    Committed,
    Abandoned,
}

impl CommitPhase {
    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Idle => "idle",
            Self::WaitMidcourt => "wait midcourt",
            Self::WaitWindow => "wait window",
            Self::InWindow => "in window",
            Self::TooLate => "too late",
            Self::Committed => "committed",
            Self::Abandoned => "abandoned",
        };
    }

    /// Status 창용 한글 설명.
    pub fn label_ko(self) -> &'static str {
        return match self {
            Self::Idle => "대기",
            Self::WaitMidcourt => "상대 코트 — 대기 중",
            Self::WaitWindow => "아직 이름 — 창 대기",
            Self::InWindow => "스윙 결정 가능",
            Self::TooLate => "너무 늦음",
            Self::Committed => "스윙 확정",
            Self::Abandoned => "이번 공 포기",
        };
    }
}
