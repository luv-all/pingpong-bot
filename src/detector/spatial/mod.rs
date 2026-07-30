//! 고정 캠 테이블 옆변 투영 → 바닥 마스크 · 공 면적 밴드.

mod ball_area;
mod floor_edge;

pub(crate) use ball_area::scorer_params_from_calib;
pub use floor_edge::{Axis, CutEdge, FloorEdgeMask};
