//! 캘리브·라이브 캠 CLI **요청** — [`Default`]가 앱 프리셋.
//!
//! datasheet(B0332)는 [`crate::constants::camera`]. USB device·보드 치수는 여기.
//! 비전 산출물 JSON은 [`DEFAULT_DATA_DIR`] 아래 — calib·colormask 툴 전부 여기만.

use std::path::{Path, PathBuf};

use crate::camera::{
    CamCliArgs, CamRigConfig, CamStreamArgs, CameraId, CameraRole, CharucoBoardSpec,
    StereoCamCliArgs, StereoPairCliArgs,
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

/// 캘리브·colormask 등 비전 산출물 루트.
pub const DEFAULT_DATA_DIR: &str = "data";
/// 멀티캠 `Calibration` 번들 SSOT.
pub const DEFAULT_CALIBRATION_PATH: &str = "data/calibration.json";
/// table-PnP accepted 해 사이드카 파일명 (`-o` 부모 또는 [`DEFAULT_DATA_DIR`]).
pub const DEFAULT_CALIBRATION_PENDING_NAME: &str = "calibration.pending.json";
/// 카메라별 `ColormaskParams` 번들 SSOT.
pub const DEFAULT_COLORMASK_PATH: &str = "data/colormask.json";
/// `record-stereo` 오프라인 클립 루트 (`{scene}_{nn}/left.avi` …).
pub const DEFAULT_CLIPS_DIR: &str = "data/clips";

/// [`DEFAULT_CALIBRATION_PATH`]의 `PathBuf`.
pub fn calibration_path() -> PathBuf {
    return PathBuf::from(DEFAULT_CALIBRATION_PATH);
}

/// [`DEFAULT_COLORMASK_PATH`]의 `PathBuf`.
pub fn colormask_path() -> PathBuf {
    return PathBuf::from(DEFAULT_COLORMASK_PATH);
}

/// `-o`가 있으면 그 부모 옆 pending, 없으면 [`DEFAULT_DATA_DIR`] 아래.
pub fn calibration_pending_path(output: Option<&Path>) -> PathBuf {
    if let Some(output) = output {
        if let Some(parent) = output.parent() {
            if !parent.as_os_str().is_empty() {
                return parent.join(DEFAULT_CALIBRATION_PENDING_NAME);
            }
        }
    }
    return PathBuf::from(DEFAULT_DATA_DIR).join(DEFAULT_CALIBRATION_PENDING_NAME);
}

/// `path`의 부모 디렉터리를 만든다 (`data/` 등).
pub fn ensure_parent_dir(path: &Path) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            return std::fs::create_dir_all(parent);
        }
    }
    return Ok(());
}

/// 벤치 스테레오 리그 — USB 순서가 바뀌면 **여기만** 고친다.
pub const LEFT_DEVICE: i32 = 0;
pub const RIGHT_DEVICE: i32 = 1;
pub const LEFT_CAMERA_ID: u8 = 0;
pub const RIGHT_CAMERA_ID: u8 = 1;

pub const MAX_REPROJ_RMSE_PX: f64 = 7.0;
pub const MIN_CHARUCO_CORNERS: usize = 4;

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
            cam: Vec::new(),
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

impl Default for StereoPairCliArgs {
    fn default() -> Self {
        return Self {
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
