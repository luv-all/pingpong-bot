//! 스윙 인터셉트 창.

use crate::planner::InterceptWindow;

pub fn intercept() -> InterceptWindow {
    // rail_frame behind≈0.10 기준 접수 창.
    // (더 뒤 behind≈0.20일 때는 앞쪽만 닿아 0.0..0.18로 좁혔었음.)
    return InterceptWindow {
        y_min: 0.08,
        y_max: 0.35,
        sample_step: 0.03,
    };
}
