//! 임의 revolute 직렬 체인의 기구학 표현.

mod chain;
mod error;
mod joint;

pub use chain::SerialChain;
pub use error::SerialChainError;
pub use joint::SerialJoint;
