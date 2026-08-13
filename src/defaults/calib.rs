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
/// 2026-08-12 에 옛 세대 클립(`fly_10`~`fly_20`과 그 전부, 커밋 `8d22896`)을 지우고
/// `fly_45`~`fly_54`(실서브 — 상대 쪽 1바운스 후 로봇 쪽 바운스 전에 인터셉트)로
/// 교체했다. 캘리브는 안 옮겼으니 `data/calibration.json`/`colormask.json`은 그대로
/// 맞는다. 옛 슈터-피드 클립(바운스 0회)과 통계 성격이 다르다는 점에 주의 — 바운스가
/// 껴서 반발계수·마찰계수·스핀 오차가 그대로 드러난다.
///
/// 리그를 또 만지거나 클립을 또 갈면 여기를 갱신한다. 클립 자체에 캘리브 지문을 박는 게
/// 더 튼튼하지만(`meta.json` 에 캘리브 해시), 그건 녹화 도구를 고쳐야 해서 후속이다.
pub const CURRENT_RIG_CLIPS: [&str; 10] = [
    "fly_45", "fly_46", "fly_47", "fly_48", "fly_49", "fly_50", "fly_51", "fly_52", "fly_53",
    "fly_54",
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

/// 클립 폴더 안에 두는, **그 클립을 찍을 때의** 캘리브 스냅샷 파일명.
///
/// 클립은 과거의 기록인데 캘리브는 지금 리그를 가리키는 살아 있는 값이라, 카메라를
/// 옮기면 옛 클립이 통째로 무효가 된다. 그때 지표가 에러 없이 빈 표만 내서 두 번이나
/// 조용히 거짓말을 했다(2026-08-13: `fe94531`, `3db054e`). 찍을 때의 기하를 클립 옆에
/// 같이 두면 그 결합이 끊긴다 — 실기는 전역 파일을, 클립 도구는 클립 옆 파일을 쓴다.
pub const CLIP_CALIBRATION_NAME: &str = "calibration.json";

/// `dir` 클립이 자기 캘리브를 들고 있으면 그 경로, 없으면 전역 [`calibration_path`].
///
/// 옛 클립엔 스냅샷이 없으므로 전역으로 접는다 — 그 경우가 바로 위험한 경우라,
/// 호출 쪽이 결과가 비면 크게 실패해야 한다([`crate::camera::Calibration`] 소비자 참고).
pub fn clip_calibration_path(dir: &Path) -> PathBuf {
    let beside = dir.join(CLIP_CALIBRATION_NAME);
    if beside.is_file() {
        return beside;
    }
    return calibration_path();
}

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
