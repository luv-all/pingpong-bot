//! 클립 장면 태그.

use clap::ValueEnum;

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Scene {
    Fly,
    Roll,
    Drop,
}

impl Scene {
    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Fly => "fly",
            Self::Roll => "roll",
            Self::Drop => "drop",
        };
    }
}
