//! 카메라 입력·캘리브레이션·프레임 IO.
//!
//! - [`calib`] — `Calibration` / ChArUco / 탁구대 PnP
//! - [`io`] — 캡처 · 프리뷰 · 시뮬 카메라
//! - [`facade`] — Charuco / TablePnp / Preview
//! - [`arducam_b0332`] — B0332 datasheet (`constants::camera` re-export)
//!
//! 삼각측량 계산은 [`crate::estimator::Triangulate`].

pub mod arducam_b0332;
pub mod calib;
pub mod facade;
pub mod io;

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

pub use calib::{BoardSpec, Calibration, FrameDetect, Landmark, Pnp, PnpResult, Report};
pub use facade::{Charuco, Preview, TablePnp};
pub use io::{
    CamCliArgs, CamRigConfig, CamStreamArgs, CaptureBackend, ExposureReadout, FittedBgr, Frame,
    FrameSource, HintSource, ImageDirSource, MonoOfflineArgs, OpenCvCapture, PixelPickMouse,
    PreviewAction, ResolvedCam, ResolvedStereoOffline, ShowBgrResult, SimCamera, StereoCamCliArgs,
    StereoClip, StereoOfflineArgs, StereoPairCliArgs, StreamPreset, ThreadedCapture,
    WorldGridParams,
};
