//! 카메라 입출력 — 캡처·프리뷰·투영·시뮬 카메라.

mod cam_cli;
mod capture;
pub mod preview;
mod projection;
mod rig;
mod sim;
mod threaded;

pub use cam_cli::{
    CamCliArgs, CamStreamArgs, DEFAULT_FOV_Y_DEG, DEFAULT_STREAM_FOURCC, DEFAULT_STREAM_FPS,
    DEFAULT_STREAM_HEIGHT, DEFAULT_STREAM_WIDTH, ResolvedCam, StereoCamCliArgs, StreamPreset,
    parse_fourcc, resolve_cams,
};
pub use capture::{
    CaptureBackend, ExposureReadout, Frame, FrameSource, HintSource, ImageDirSource, OpenCvCapture,
};
pub use preview::{
    FittedBgr, PIXEL_LOUPE_SRC_HALF, PIXEL_LOUPE_ZOOM, PixelPickMouse, PreviewAction,
    ShowBgrResult, arrow_delta, destroy_window, display_fit_bounds, draw_cam_label, draw_circle_px,
    draw_debug_lines, draw_help_lines, draw_pixel_loupe, draw_world_velocity, fit_bgr_downscale,
    hstack_bgr, show_bgr, unscale_xy,
};
pub use projection::CameraView;
pub use rig::{CamRigConfig, CameraRole};
pub use sim::SimCamera;
pub use threaded::ThreadedCapture;
