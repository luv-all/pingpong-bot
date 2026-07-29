//! 카메라 캘리브레이션 — 인트린식(ChArUco) · 외참(탁구대 PnP) · 번들 JSON.

pub mod charuco;
pub mod table;

mod calibration;

pub use calibration::Calibration;
pub use charuco::{
    BoardSpec, FrameDetect, MIN_CHARUCO_CORNERS, Report, calibrate_charuco, detect_and_draw_charuco,
};
pub use table::{
    Landmark, MAX_REPROJ_RMSE_PX, Pnp, PnpResult, TABLE_LANDMARK_COUNT, calibrate_table_pnp,
    ensure_reproj_below, ensure_reproj_ok, table_landmark_mesh_edges, table_landmarks,
    upsert_camera,
};

// 移行 별칭
pub use charuco::BoardSpec as CharucoBoardSpec;
pub use charuco::FrameDetect as CharucoFrameDetect;
pub use charuco::Report as CharucoCalibReport;
pub use table::Landmark as TableLandmark;
pub use table::PnpResult as TablePnpResult;
