//! 카메라 캘리브레이션 — 인트린식(ChArUco) · 외참(탁구대 PnP) · 번들 JSON.

pub mod charuco;
pub mod table;

mod calibration;

pub use calibration::Calibration;
pub use charuco::{BoardSpec, FrameDetect, Report, calibrate_charuco, detect_and_draw_charuco};
pub use table::{
    Landmark, Pnp, PnpResult, calibrate_table_pnp, ensure_reproj_below, ensure_reproj_ok,
    table_landmark_mesh_edges, table_landmarks, upsert_camera,
};
