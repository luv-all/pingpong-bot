//! 카메라 입력·캘리브레이션·삼각측량.
//!
//! - [`calib`] — `Calibration` / ChArUco / 탁구대 PnP
//! - [`tri`] — DLT · OpenCV `triangulatePoints`
//! - [`io`] — 캡처 · 프리뷰 · 투영 · 시뮬 카메라
//! - [`arducam_b0332`] — B0332 datasheet (`constants::camera` re-export)

use std::fmt;
use std::time::Instant;

use crate::Point3;

pub mod arducam_b0332;
pub mod calib;
pub mod io;
pub mod tri;

pub use calib::{
    Calibration, CameraParams, CharucoBoardSpec, CharucoCalibReport, CharucoFrameDetect,
    MAX_REPROJ_RMSE_PX, MIN_CHARUCO_CORNERS, TABLE_LANDMARK_COUNT, TableLandmark, TablePnpResult,
};
pub use io::{
    CamCliArgs, CamRigConfig, CamStreamArgs, CameraRole, CaptureBackend, DEFAULT_CLIPS_DIR,
    DEFAULT_FOV_Y_DEG, DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS, DEFAULT_STREAM_HEIGHT,
    DEFAULT_STREAM_WIDTH, ExposureReadout, FittedBgr, Frame, FrameSource, HintSource,
    ImageDirSource, MonoOfflineArgs, OpenCvCapture, PixelPickMouse, PreviewAction, ResolvedCam,
    ResolvedStereoOffline, ShowBgrResult, SimCamera, StereoCamCliArgs, StereoClip,
    StereoOfflineArgs, StereoPairCliArgs, StreamPreset, ThreadedCapture, WorldGridParams,
};

/// 이미지 픽셀 좌표.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PixelPoint {
    pub x: f64,
    pub y: f64,
}

impl PixelPoint {
    pub fn new(x: f64, y: f64) -> Self {
        return Self { x, y };
    }

    pub fn lerp(self, other: Self, w: f64) -> Self {
        return Self {
            x: self.x + (other.x - self.x) * w,
            y: self.y + (other.y - self.y) * w,
        };
    }
}

/// 카메라 식별자.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub struct CameraId(pub u8);

impl CameraId {
    pub const fn new(index: u8) -> Self {
        return Self(index);
    }
}

impl fmt::Display for CameraId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "카메라 {}번", self.0);
    }
}

/// 한 프레임에서 검출한 공.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallObservation {
    pub pixel: PixelPoint,
    pub camera_id: CameraId,
    pub timestamp: Instant,
}

/// ChArUco 보정 공개 진입점.
pub struct Charuco;

impl Charuco {
    pub fn calibrate(
        dir: &std::path::Path,
        board_spec: CharucoBoardSpec,
        camera_id: CameraId,
    ) -> Result<(Calibration, CharucoCalibReport), String> {
        return calib::calibrate_charuco(dir, board_spec, camera_id);
    }

    pub fn detect_and_draw(
        bgr: &opencv::core::Mat,
        board_spec: CharucoBoardSpec,
    ) -> Result<(opencv::core::Mat, CharucoFrameDetect), String> {
        return calib::detect_and_draw_charuco(bgr, board_spec);
    }
}

/// 탁구대 solvePnP 공개 진입점.
pub struct TablePnp;

impl TablePnp {
    pub fn calibrate(
        camera_id: CameraId,
        label: Option<String>,
        width: u32,
        height: u32,
        fov_y_deg: f64,
        pixels: &[PixelPoint],
    ) -> Result<TablePnpResult, String> {
        return calib::calibrate_table_pnp(camera_id, label, width, height, fov_y_deg, pixels);
    }

    pub fn ensure_reproj_below(result: &TablePnpResult, max_rmse: f64) -> Result<(), String> {
        return calib::ensure_reproj_below(result, max_rmse);
    }

    pub fn ensure_reproj_ok(result: &TablePnpResult) -> Result<(), String> {
        return calib::ensure_reproj_ok(result);
    }

    pub fn upsert_camera(calibration: &mut Calibration, params: CameraParams) {
        calib::upsert_camera(calibration, params);
    }

    pub fn landmarks() -> [TableLandmark; TABLE_LANDMARK_COUNT] {
        return calib::table_landmarks();
    }

    pub fn landmark_mesh_edges() -> &'static [(usize, usize)] {
        return calib::table_landmark_mesh_edges();
    }
}

/// 삼각측량 공개 진입점.
pub struct Triangulate;

impl Triangulate {
    pub fn sample_at(observations: &[BallObservation], sync_time: Instant) -> Option<PixelPoint> {
        return tri::sample_at(observations, sync_time);
    }

    pub fn synced(
        observations_by_camera: &[(CameraId, &[BallObservation])],
        sync_time: Instant,
        calibration: &Calibration,
    ) -> Result<Point3, crate::DomainError> {
        return tri::triangulate_synced(observations_by_camera, sync_time, calibration);
    }

    pub fn views(views: &[(nalgebra::Matrix3x4<f64>, PixelPoint)]) -> Option<Point3> {
        return tri::triangulate_views(views);
    }

    /// 캘리브 + 픽셀 히트 → 월드 점. 카메라 수·params 부족하면 `None`.
    pub fn pixels(hits: &[(CameraId, PixelPoint)], calibration: &Calibration) -> Option<Point3> {
        if hits.len() < calibration.min_cameras_for_triangulation() {
            return None;
        }
        let views: Vec<_> = hits
            .iter()
            .map(|&(id, pix)| {
                calibration
                    .params(id)
                    .map(|params| (params.projection_matrix(), pix))
            })
            .collect::<Option<_>>()?;
        return Self::views(&views);
    }

    pub fn dlt(views: &[(nalgebra::Matrix3x4<f64>, PixelPoint)]) -> Option<Point3> {
        return tri::dlt_triangulate(views);
    }

    pub fn projections(
        calibration: &Calibration,
        camera_ids: &[CameraId],
        point: Point3,
    ) -> Option<Point3> {
        return tri::triangulate_projections(calibration, camera_ids, point);
    }
}

/// OpenCV 프리뷰/오버레이 공개 진입점.
pub struct Preview;

impl Preview {
    pub fn fit_bgr_downscale(
        image: &opencv::core::Mat,
        max_w: i32,
        max_h: i32,
    ) -> opencv::Result<io::FittedBgr> {
        return io::fit_bgr_downscale(image, max_w, max_h);
    }

    pub fn unscale_xy(x: i32, y: i32, scale: f64) -> (i32, i32) {
        return io::unscale_xy(x, y, scale);
    }

    pub fn display_fit_bounds() -> Option<(i32, i32)> {
        return io::display_fit_bounds();
    }

    pub fn show_bgr(
        window: &str,
        image: &opencv::core::Mat,
        wait_ms: i32,
    ) -> opencv::Result<ShowBgrResult> {
        return io::show_bgr(window, image, wait_ms);
    }

    pub fn destroy_window(window: &str) {
        io::destroy_window(window);
    }

    pub fn hstack_bgr(panels: &[opencv::core::Mat]) -> opencv::Result<opencv::core::Mat> {
        return io::hstack_bgr(panels);
    }

    pub fn draw_debug_lines(
        img: &mut opencv::core::Mat,
        lines: &[impl AsRef<str>],
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return io::draw_debug_lines(img, lines, color);
    }

    pub fn draw_help_lines(
        img: &mut opencv::core::Mat,
        lines: &[impl AsRef<str>],
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return io::draw_help_lines(img, lines, color);
    }

    pub fn draw_circle_px(
        img: &mut opencv::core::Mat,
        pixel: PixelPoint,
        radius_px: i32,
        color: opencv::core::Scalar,
        thickness: i32,
    ) -> opencv::Result<()> {
        return io::draw_circle_px(img, pixel, radius_px, color, thickness);
    }

    pub fn draw_world_velocity(
        img: &mut opencv::core::Mat,
        params: &CameraParams,
        origin: Point3,
        velocity: nalgebra::Vector3<f64>,
        scale_secs: f64,
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return io::draw_world_velocity(img, params, origin, velocity, scale_secs, color);
    }

    pub fn draw_world_grid(
        img: &mut opencv::core::Mat,
        params: &CameraParams,
        grid: &WorldGridParams,
    ) -> opencv::Result<()> {
        return io::draw_world_grid(img, params, *grid);
    }

    pub fn apply_grid_key(grid: &mut WorldGridParams, key: i32) {
        io::apply_grid_key(grid, key);
    }

    pub fn draw_cam_label(
        img: &mut opencv::core::Mat,
        label: &str,
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return io::draw_cam_label(img, label, color);
    }

    pub fn arrow_delta(key: i32) -> Option<(i32, i32)> {
        return io::arrow_delta(key);
    }

    pub fn draw_pixel_loupe(
        dst: &mut opencv::core::Mat,
        src: &opencv::core::Mat,
        cx: i32,
        cy: i32,
    ) -> opencv::Result<()> {
        return io::draw_pixel_loupe(dst, src, cx, cy);
    }
}
