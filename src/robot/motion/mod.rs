//! 접수 계획 — 임팩트 역산 · 인터셉트 창 · quintic/bang-bang 궤적.
//!
//! 파이프라인 세 번째 단계: [`crate::detector`] → [`crate::estimator`] → `motion`.
//! 예측된 공 궤적을 받아 **어디서·언제·어떻게** 받아칠지 정하고 관절 궤적을 낸다.
//!
//! 팔·테이블 기하 제약(관통 판정)은 로봇 소유 — [`crate::robot::collision`].

pub mod bang_bang;
pub mod feasibility;
pub mod fixed_swing;
pub mod impact_candidate;
pub mod impact_target;
pub mod intercept_window;
pub mod physics;
pub mod planned_intercept;
pub mod planner;
pub mod quintic_segment;
pub mod rail;
pub mod trajectory;

pub use bang_bang::{RacketGuidanceScratch, RacketGuidanceStep};
pub use feasibility::Feasibility;
pub use fixed_swing::{
    DEFAULT_IMPACT_TIME_STRATEGY, DEFAULT_SWING_SHAPE_STRATEGY, FIXED_SWING_START_DEG,
    ImpactTimeStrategy, SwingHeightBand, SwingShapeStrategy, fixed_swing_end_joints,
    fixed_swing_impact_time_secs, fixed_swing_rail_target, fixed_swing_start_joints,
    should_start_fixed_swing,
};
pub use intercept_window::InterceptWindow;
pub use planned_intercept::PlannedIntercept;
pub use planner::Planner;
pub use quintic_segment::QuinticSegment;
pub use rail::Rail;
pub use trajectory::Trajectory;
