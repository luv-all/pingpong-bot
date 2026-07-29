//! 탁구대 랜드마크 · solvePnP 외참.

mod landmark;
mod landmarks;
mod pnp;
mod pnp_result;

pub use landmark::Landmark;
pub use landmarks::{
    MAX_REPROJ_RMSE_PX, TABLE_LANDMARK_COUNT, table_landmark_mesh_edges, table_landmarks,
};
pub use pnp::{calibrate_table_pnp, ensure_reproj_below, ensure_reproj_ok, upsert_camera};
pub use pnp_result::PnpResult;

/// 탁구대 PnP 연산 마커 (자유함수는 이 모듈에 둠).
pub struct Pnp;
