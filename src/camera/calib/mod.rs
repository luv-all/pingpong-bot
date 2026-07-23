//! 카메라 캘리브레이션 — 인트린식(ChArUco) · 외참(탁구대 PnP) · 번들 JSON.

mod calibration;
mod charuco;
mod table_landmarks;
mod table_pnp;

pub use calibration::{Calibration, CameraParams};
pub use charuco::{
    CharucoBoardSpec, CharucoCalibReport, CharucoFrameDetect, MIN_CHARUCO_CORNERS,
    calibrate_charuco, detect_and_draw_charuco,
};
pub use table_landmarks::{
    MAX_REPROJ_RMSE_PX, TABLE_LANDMARK_COUNT, TableLandmark, table_landmark_mesh_edges,
    table_landmarks,
};
pub use table_pnp::{
    TablePnpResult, calibrate_table_pnp, ensure_reproj_below, ensure_reproj_ok, upsert_camera,
};
