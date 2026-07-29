//! 카메라 식별자.

use std::fmt;

/// 카메라 식별자.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub struct Id(pub u8);

impl Id {
    pub const fn new(index: u8) -> Self {
        return Self(index);
    }
}

impl fmt::Display for Id {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "카메라 {}번", self.0);
    }
}
