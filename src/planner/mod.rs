//! 스윙/충돌/임팩트/관절 궤적 계획 중 planner 레이어 잔여물.
//!
//! 스윙 도메인은 [`crate::swing`] — 여기엔 임팩트 역산·충돌·인터셉트 창만 둔다.

pub mod collision;
pub mod impact;
pub mod intercept_window;

pub use collision::OrientedBox;
pub use impact::Impact;
pub use intercept_window::{InterceptWindow, MAX_INTERCEPT_SAMPLES};
