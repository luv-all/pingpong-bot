//! 스윙 계획 — 탄도 후보 평가 + quintic 궤적.

pub mod physics;
pub mod trajectory;

pub use physics::{
    PlannedIntercept, SwingFeasibility, accel, aero_accel, ball_past_midcourt_for_commit,
    in_swing_commit_window, plan_best_swing, plan_coarse_track, plan_return_to_center, plan_swing,
    swing_feasibility,
};
pub use trajectory::{RailMotion, SwingTrajectory};
