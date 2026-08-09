//! Appearance → 후보 목록.

use crate::camera::Frame;
use crate::detector::Candidate;

/// 프레임 → 공 후보 목록.
pub trait CandidateGenerator: Send {
    fn generate(&mut self, frame: &Frame) -> Vec<Candidate>;
}
