//! 평가 발사 순서.

/// 평가 발사 순서.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// 왼쪽 10 → 중앙 10 → 오른쪽 10.
    #[default]
    Block,
    /// 왼→중앙→오→중앙→왼→… (존당 10발될 때까지).
    Alternating,
}

impl Mode {
    pub fn label(self) -> &'static str {
        return match self {
            Self::Block => "Block (L×10→C×10→R×10)",
            Self::Alternating => "Alternating (L→C→R→C→…)",
        };
    }

    pub fn short_label(self) -> &'static str {
        return match self {
            Self::Block => "Block",
            Self::Alternating => "Alt",
        };
    }
}
