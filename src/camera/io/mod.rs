//! 카메라 입출력 — 캡처·프리뷰·시뮬 카메라.

mod cam_cli;
mod capture;
mod clip;
pub mod preview;
mod rig;
mod sim;
mod threaded;

pub use cam_cli::{
    CamCliArgs, CamStreamArgs, MonoOfflineArgs, ResolvedCam, StereoCamCliArgs, StereoOfflineArgs,
    StereoPairCliArgs, StreamPreset,
};
pub use capture::{
    CaptureBackend, ExposureReadout, Frame, FrameSource, HintSource, ImageDirSource, OpenCvCapture,
};
pub use clip::{ResolvedStereoOffline, StereoClip};
pub use preview::{
    FittedBgr, PIXEL_LOUPE_SRC_HALF, PIXEL_LOUPE_ZOOM, PixelPickMouse, PreviewAction,
    ShowBgrResult, WorldGridParams, apply_grid_key, arrow_delta, destroy_window,
    display_fit_bounds, draw_cam_label, draw_circle_px, draw_debug_lines, draw_help_lines,
    draw_pixel_loupe, draw_world_grid, draw_world_velocity, fit_bgr_downscale, hstack_bgr,
    show_bgr, unscale_xy,
};
pub use rig::CamRigConfig;
pub use sim::SimCamera;
pub use threaded::ThreadedCapture;
