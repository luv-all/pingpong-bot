//! 카메라 입력·캘리브레이션·프레임 IO.
//!
//! - [`calib`] — `Calibration` / ChArUco / 탁구대 PnP
//! - [`io`] — 캡처 · 프리뷰 · 시뮬 카메라
//! - [`facade`] — Charuco / TablePnp / Preview
//! - [`arducam_b0332`] — B0332 datasheet (`constants::camera` re-export)
//!
//! 삼각측량은 [`Triangulate`].

pub mod arducam_b0332;
pub mod calib;
pub mod facade;
pub mod io;

mod id;
mod params;
mod role;
mod triangulate;
mod view;

/// 이미지 픽셀 좌표.
///
/// [`crate::Point3`]와 같은 방식의 별칭이다 — 뺄셈이 `Vector2`가 되고 `norm`·`lerp`가
/// 딸려온다. 자체 구조체를 두면 외부 타입이 아니라서 편할 것 같지만, 실제로는
/// `dx.hypot(dy)`를 손으로 다시 쓰게 된다.
pub type Pixel = nalgebra::Point2<f64>;

pub use id::Id;
pub use params::Params;
pub use role::Role;
pub use triangulate::Triangulate;
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
