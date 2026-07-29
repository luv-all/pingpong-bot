//! 카메라 입력·캘리브레이션·삼각측량.
//!
//! - [`calib`] — `Calibration` / ChArUco / 탁구대 PnP
//! - [`tri`] — DLT · OpenCV `triangulatePoints`
//! - [`io`] — 캡처 · 프리뷰 · 투영 · 시뮬 카메라
//! - [`facade`] — Charuco / TablePnp / Triangulate / Preview
//! - [`arducam_b0332`] — B0332 datasheet (`constants::camera` re-export)

pub mod arducam_b0332;
pub mod calib;
pub mod facade;
pub mod io;
pub mod tri;

mod id;
mod params;
mod pixel;
mod role;
mod view;

pub use id::Id;
pub use params::Params;
pub use pixel::Pixel;
pub use role::Role;
pub use view::View;

pub use calib::{
    BoardSpec, Calibration, FrameDetect, Landmark, MAX_REPROJ_RMSE_PX, MIN_CHARUCO_CORNERS, Pnp,
    PnpResult, Report, TABLE_LANDMARK_COUNT,
};
pub use facade::{Charuco, Preview, TablePnp, Triangulate};
pub use io::{
    CamCliArgs, CamRigConfig, CamStreamArgs, CaptureBackend, DEFAULT_CLIPS_DIR, DEFAULT_FOV_Y_DEG,
    DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS, DEFAULT_STREAM_HEIGHT, DEFAULT_STREAM_WIDTH,
    ExposureReadout, FittedBgr, Frame, FrameSource, HintSource, ImageDirSource, MonoOfflineArgs,
    OpenCvCapture, PixelPickMouse, PreviewAction, ResolvedCam, ResolvedStereoOffline,
    ShowBgrResult, SimCamera, StereoCamCliArgs, StereoClip, StereoOfflineArgs, StereoPairCliArgs,
    StreamPreset, ThreadedCapture, WorldGridParams,
};
