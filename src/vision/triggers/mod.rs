//! [`Trigger`](super::Trigger) 구현들.

mod bounce;
mod combine;
mod plane;
mod sigma;

pub use bounce::FirstBounce;
pub use combine::{All, Any};
pub use plane::PlaneCrossing;
pub use sigma::SigmaThreshold;

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
