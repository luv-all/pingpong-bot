//! 점수·후보 — candidate · scorer.

pub mod candidate;
pub mod params;
pub mod scorer;

pub use candidate::Candidate;
pub use params::ScorerParams;
pub use scorer::Scorer;
