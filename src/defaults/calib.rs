//! 캘리브·라이브 캠 CLI **요청** — [`Default`]가 앱 프리셋.
//!
//! datasheet(B0332)는 [`crate::constants::camera`]. USB device·보드 치수는 여기.
//! Calibration JSON 경로는 [`DEFAULT_CALIBRATION_PATH`] — 저장·로드 툴 전부 여기만.

use std::path::{Path, PathBuf};

use crate::camera::{
    CamCliArgs, CamRigConfig, CamStreamArgs, CameraId, CameraRole, CharucoBoardSpec,
    StereoCamCliArgs,
};
use crate::constants::camera::arducam_b0332;

/// OpenCV 백엔드 CLI 기본 (`recommended` → OS별 CaptureBackend::recommended).
pub const DEFAULT_STREAM_BACKEND: &str = "recommended";
pub const DEFAULT_STREAM_WIDTH: i32 = arducam_b0332::WIDTH;
pub const DEFAULT_STREAM_HEIGHT: i32 = arducam_b0332::HEIGHT;
pub const DEFAULT_STREAM_FPS: f64 = arducam_b0332::FPS_MJPG;
pub const DEFAULT_STREAM_FOURCC: &str = arducam_b0332::FOURCC_MJPG;
/// 라이브 UI와 캡처 분리 (hinguri grab 스레드와 동일 계열). 끄려면 `--threaded=false`.
pub const DEFAULT_STREAM_THREADED: bool = true;
pub const DEFAULT_FOV_Y_DEG: f64 = arducam_b0332::VFOV_DEG;

/// 멀티캠 `Calibration` 번들 SSOT. calib 저장·measure 로드 기본 경로.
pub const DEFAULT_CALIBRATION_PATH: &str = "calibration.json";
/// table-PnP accepted 해 사이드카 (본파일 upsert 전). `-o` 옆 또는 CWD.
pub const DEFAULT_CALIBRATION_PENDING_NAME: &str = "calibration.pending.json";

/// [`DEFAULT_CALIBRATION_PATH`]의 `PathBuf`.
pub fn calibration_path() -> PathBuf {
    return PathBuf::from(DEFAULT_CALIBRATION_PATH);
}

/// `-o`가 있으면 그 부모 옆 pending, 없으면 CWD의 [`DEFAULT_CALIBRATION_PENDING_NAME`].
pub fn calibration_pending_path(output: Option<&Path>) -> PathBuf {
    if let Some(output) = output {
        if let Some(parent) = output.parent() {
            if !parent.as_os_str().is_empty() {
                return parent.join(DEFAULT_CALIBRATION_PENDING_NAME);
            }
        }
    }
    return PathBuf::from(DEFAULT_CALIBRATION_PENDING_NAME);
}

/// 벤치 스테레오 리그 — USB 순서가 바뀌면 **여기만** 고친다.
pub const LEFT_DEVICE: i32 = 0;
pub const RIGHT_DEVICE: i32 = 1;
pub const LEFT_CAMERA_ID: u8 = 0;
pub const RIGHT_CAMERA_ID: u8 = 1;

pub const MAX_REPROJ_RMSE_PX: f64 = 7.0;
pub const MIN_CHARUCO_CORNERS: usize = 4;

pub const DEFAULT_CAM_ROLES: [CameraRole; 1] = [CameraRole::Left];
pub const DEFAULT_STEREO_CAM_ROLES: [CameraRole; 2] = [CameraRole::Left, CameraRole::Right];

pub const CHARUCO_SQUARES_X: i32 = 5;
pub const CHARUCO_SQUARES_Y: i32 = 7;
pub const CHARUCO_SQUARE_LENGTH_M: f32 = 0.04;
pub const CHARUCO_MARKER_LENGTH_M: f32 = 0.02;

impl Default for CamStreamArgs {
    fn default() -> Self {
        return Self {
            backend: DEFAULT_STREAM_BACKEND.into(),
            width: DEFAULT_STREAM_WIDTH,
            height: DEFAULT_STREAM_HEIGHT,
            fps: DEFAULT_STREAM_FPS,
            fourcc: DEFAULT_STREAM_FOURCC.into(),
            threaded: DEFAULT_STREAM_THREADED,
            preset: None,
        };
    }
}

impl Default for CamRigConfig {
    fn default() -> Self {
        return Self {
            left_device: LEFT_DEVICE,
            right_device: RIGHT_DEVICE,
            left_id: CameraId(LEFT_CAMERA_ID),
            right_id: CameraId(RIGHT_CAMERA_ID),
        };
    }
}

impl Default for CamCliArgs {
    fn default() -> Self {
        return Self {
            cam: DEFAULT_CAM_ROLES.to_vec(),
            stream: CamStreamArgs::default(),
        };
    }
}

impl Default for StereoCamCliArgs {
    fn default() -> Self {
        return Self {
            cam: DEFAULT_STEREO_CAM_ROLES.to_vec(),
            stream: CamStreamArgs::default(),
        };
    }
}

impl Default for CharucoBoardSpec {
    fn default() -> Self {
        return Self {
            squares_x: CHARUCO_SQUARES_X,
            squares_y: CHARUCO_SQUARES_Y,
            square_length_m: CHARUCO_SQUARE_LENGTH_M,
            marker_length_m: CHARUCO_MARKER_LENGTH_M,
        };
    }
}
