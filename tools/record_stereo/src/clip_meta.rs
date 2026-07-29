//! 클립 meta.json.

use serde::Serialize;

#[derive(Serialize)]
pub struct ClipMeta {
    pub scene: String,
    pub preroll_secs: f64,
    pub postroll_secs: f64,
    pub width: i32,
    pub height: i32,
    pub request_fps: f64,
    pub meas_fps: f64,
    pub writer_fps: f64,
    pub fourcc: String,
    pub backend: String,
    pub frames: usize,
    pub created_unix_secs: u64,
}
