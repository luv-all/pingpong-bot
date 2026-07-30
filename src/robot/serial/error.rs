//! 직렬 체인 오류.

use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialChainError {
    Empty,
    InvalidAxis,
}

impl fmt::Display for SerialChainError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return match self {
            Self::Empty => write!(f, "직렬 체인에 revolute 관절이 없습니다"),
            Self::InvalidAxis => write!(f, "revolute 관절 축이 유효하지 않습니다"),
        };
    }
}

impl std::error::Error for SerialChainError {}
