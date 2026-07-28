//! 고정 캠 테이블 옆변(x=0 / x=W) 투영 → 바닥쪽 사다리꼴 마스크.

mod floor_edge;
mod gate;
mod ball_area;

pub use ball_area::scorer_params_from_calib;
pub use floor_edge::FloorEdgeMask;
pub use gate::SpatialGate;
