//! 캘리브·라이브 캠 CLI **요청** — [`Default`]가 앱 프리셋.
//!
//! datasheet(B0332)는 [`crate::constants::camera`]. USB device·보드 치수는 여기.
//! 비전 산출물 JSON은 [`DEFAULT_DATA_DIR`] 아래 — calib·colormask 툴 전부 여기만.

use crate::camera;
use std::path::{Path, PathBuf};

use crate::camera::{CamCliArgs, CamRigConfig, CamStreamArgs, StereoCamCliArgs, StereoPairCliArgs};
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

/// 지금 리그로 찍은 클립. **이보다 오래된 클립은 쓰면 안 된다.**
///
/// 2026-08-03 에 카메라를 옮기고 다시 캘리브했다. 그 전 클립(`fly_01`~`fly_09`, `drop_02`,
/// `roll_01`)은 지금 `data/calibration.json` 과 맞지 않는다 — 같은 픽셀이 다른 3D 점을
/// 가리키므로 지표가 조용히 거짓말을 한다. 파일은 남겨 두되 도구가 자동으로 집지 않는다.
///
/// 리그를 또 만지면 여기를 갱신한다. 클립 자체에 캘리브 지문을 박는 게 더 튼튼하지만
/// (`meta.json` 에 캘리브 해시), 그건 녹화 도구를 고쳐야 해서 후속이다.
pub const CURRENT_RIG_CLIPS: [&str; 11] = [
    "fly_10", "fly_11", "fly_12", "fly_13", "fly_14", "fly_15", "fly_16", "fly_17", "fly_18",
    "fly_19", "fly_20",
];

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
pub const LEFT_DEVICE: i32 = 1;
pub const RIGHT_DEVICE: i32 = 0;
/// `data/calibration.json`·`data/colormask.json`의 `camera_id`와 맞춘 값.
///
/// 예전엔 1/0(뒤집힘)이었다 — `clip_review`는 `left.avi`를 `camera::Id(0)`로 고정해 읽는데
/// (`tools/clip_review/src/main.rs`), 여기(`CamRigConfig`를 타는 `--cam left|right` 툴들)는
/// `Id(1)`을 썼다. 월드 격자 투영이 `Id(0)`일 때만 `left.avi`의 테이블 모서리·네트에
/// 픽셀 단위로 붙어서(2026-08-11, `clip-review --grid`) 뒤집혔던 쪽이 여기였다고 확인,
/// 바로잡음 — `detect-full --cam left`가 엉뚱한 캘리브·colormask를 물던 원인.
pub const LEFT_CAMERA_ID: u8 = 0;
pub const RIGHT_CAMERA_ID: u8 = 1;

pub const MAX_REPROJ_RMSE_PX: f64 = 8.5;
pub const MIN_CHARUCO_CORNERS: usize = 4;

pub const DEFAULT_STEREO_CAM_ROLES: [camera::Role; 2] = [camera::Role::Left, camera::Role::Right];

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
            left_id: camera::Id(LEFT_CAMERA_ID),
            right_id: camera::Id(RIGHT_CAMERA_ID),
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

impl Default for camera::BoardSpec {
    fn default() -> Self {
        return Self {
            squares_x: CHARUCO_SQUARES_X,
            squares_y: CHARUCO_SQUARES_Y,
            square_length_m: CHARUCO_SQUARE_LENGTH_M,
            marker_length_m: CHARUCO_MARKER_LENGTH_M,
        };
    }
}
