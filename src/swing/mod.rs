//! 스윙 계획 — quintic·bang-bang·실현가능성.

pub mod bang_bang;
pub mod feasibility;
pub mod impact_candidate;
pub mod impact_target;
pub mod physics;
pub mod planned_intercept;
pub mod planner;
pub mod quintic_segment;
pub mod rail_motion;
pub mod trajectory;

pub use bang_bang::{RacketGuidanceScratch, RacketGuidanceStep};
pub use feasibility::Feasibility;
pub use planned_intercept::PlannedIntercept;
pub use planner::Planner;
pub use quintic_segment::QuinticSegment;
pub use rail_motion::RailMotion;
pub use trajectory::Trajectory;
