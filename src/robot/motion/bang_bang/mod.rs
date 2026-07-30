//! 순수 토크 bang-bang 스윙 (디버그/벤치 경로).

pub mod guidance;
pub mod planned_intercept;
pub mod racket_guidance_scratch;
pub mod racket_guidance_step;
pub mod trajectory;

pub use guidance::step_racket_guidance;
pub use planned_intercept::{PlannedIntercept, plan_bang_bang_swing};
pub use racket_guidance_scratch::RacketGuidanceScratch;
pub use racket_guidance_step::RacketGuidanceStep;
pub use trajectory::Trajectory;
