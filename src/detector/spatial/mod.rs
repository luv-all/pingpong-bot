//! 고정 캠 테이블 투영 → 공간 keep 마스크 · 공 면적 밴드.

mod ball_area;
mod floor_edge;
mod spatial_mask;
mod table_corridor;

pub(crate) use ball_area::scorer_params_from_calib;
pub use floor_edge::FloorEdgeMask;
pub use spatial_mask::SpatialMask;
pub use table_corridor::TableCorridorMask;
