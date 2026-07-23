//! 스윙 인터셉트 창.

use crate::planner::InterceptWindow;

pub fn intercept() -> InterceptWindow {
    // 철제 프로파일 마운트(y≈−0.20) 기준 도달 가능한 접수 구간.
    return InterceptWindow {
        y_min: 0.0,
        y_max: 0.18,
        sample_step: 0.03,
    };
}
