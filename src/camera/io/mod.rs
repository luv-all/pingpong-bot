//! 카메라 입출력 — 캡처·프리뷰·투영·시뮬 카메라.

mod capture;
pub mod preview;
mod projection;
mod sim;

pub use capture::{
    ExposureReadout, Frame, FrameSource, HintSource, ImageDirSource, OpenCvCapture,
};
pub use preview::{
    PreviewAction, destroy_window, draw_cam_label, draw_circle_px, draw_debug_lines,
    draw_help_lines, draw_world_velocity, hstack_bgr, show_bgr,
};
pub use projection::CameraView;
pub use sim::SimCamera;
