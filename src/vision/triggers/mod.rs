//! [`Trigger`](super::Trigger) 구현들.

mod bounce;
mod combine;
mod plane;
mod sigma;
mod stereo;

pub use bounce::FirstBounce;
pub use combine::{All, Any};
pub use plane::PlaneCrossing;
pub use sigma::SigmaThreshold;
pub use stereo::StereoSamples;

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
