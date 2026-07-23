//! Fuse 레이어 — candidate · scorer · fuse 조립.

pub mod candidate;
pub mod fuse;
pub mod params;
pub mod scorer;

pub use candidate::Candidate;
pub use fuse::{CandidateGenerator, FuseDetector, IntoCandidateGenerators, fuse};
pub use params::{RoiParams, ScorerParams};
pub use scorer::Scorer;
