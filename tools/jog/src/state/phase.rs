//! Sync / Apply / Discard 단계.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Sync 필요 (시작 직후 / Apply 후).
    NeedsSync,
    /// 미리보기 1회 가능.
    Ready,
    /// 스테이징됨 — Apply 또는 Discard.
    Previewed,
    /// Apply 직후 — 수동 Sync 필수.
    AwaitingSync,
}

impl Phase {
    pub fn label(self) -> &'static str {
        return match self {
            Self::NeedsSync => "동기화 필요",
            Self::Ready => "준비",
            Self::Previewed => "미리보기",
            Self::AwaitingSync => "동기화 필요",
        };
    }

    pub fn can_preview(self) -> bool {
        return matches!(self, Self::Ready);
    }

    pub fn can_apply(self) -> bool {
        return matches!(self, Self::Previewed);
    }

    pub fn can_discard(self) -> bool {
        return matches!(self, Self::Previewed);
    }

    pub fn can_sync(self) -> bool {
        return true;
    }
}
