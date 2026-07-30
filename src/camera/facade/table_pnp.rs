//! 탁구대 solvePnP 공개 진입점.

use crate::camera;
use crate::camera::calib::{
    Calibration, Landmark, PnpResult, ensure_reproj_below, ensure_reproj_ok,
    table_landmark_mesh_edges, table_landmarks, upsert_camera,
};
use crate::constants::TABLE_LANDMARK_COUNT;

/// 탁구대 solvePnP 공개 진입점.
pub struct TablePnp;

impl TablePnp {
    pub fn calibrate(
        camera_id: camera::Id,
        label: Option<String>,
        width: u32,
        height: u32,
        fov_y_deg: f64,
        pixels: &[camera::Pixel],
    ) -> Result<PnpResult, String> {
        return crate::camera::calib::calibrate_table_pnp(
            camera_id, label, width, height, fov_y_deg, pixels,
        );
    }

    pub fn ensure_reproj_below(result: &PnpResult, max_rmse: f64) -> Result<(), String> {
        return ensure_reproj_below(result, max_rmse);
    }

    pub fn ensure_reproj_ok(result: &PnpResult) -> Result<(), String> {
        return ensure_reproj_ok(result);
    }

    pub fn upsert_camera(calibration: &mut Calibration, params: camera::Params) {
        upsert_camera(calibration, params);
    }

    pub fn landmarks() -> [Landmark; TABLE_LANDMARK_COUNT] {
        return table_landmarks();
    }

    pub fn landmark_mesh_edges() -> &'static [(usize, usize)] {
        return table_landmark_mesh_edges();
    }
}
