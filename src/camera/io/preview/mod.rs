//! OpenCV highgui 프리뷰·디버그 오버레이 (detect/measure 툴 공용).

mod fitted_bgr;
mod ops;
mod pixel_pick_mouse;
mod preview_action;
mod show_bgr_result;
mod text_block;
mod world_grid_params;

pub use fitted_bgr::{FittedBgr, fit_bgr_downscale};
pub use ops::{
    draw_cam_label, draw_circle_px, draw_rect_px, draw_text_at_px, draw_world_velocity,
    hstack_bgr, unscale_xy,
};
pub use pixel_pick_mouse::{
    PIXEL_LOUPE_SRC_HALF, PIXEL_LOUPE_ZOOM, PixelPickMouse, arrow_delta, draw_pixel_loupe,
};
pub use preview_action::PreviewAction;
pub use show_bgr_result::{ShowBgrResult, destroy_window, display_fit_bounds, show_bgr};
pub use text_block::{draw_debug_lines, draw_help_lines};
pub use world_grid_params::{WorldGridParams, apply_grid_key, draw_world_grid};
