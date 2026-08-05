//! OpenCV 프리뷰/오버레이 공개 진입점.

use crate::Point3;
use crate::camera;
use crate::camera::io::{
    FittedBgr, ShowBgrResult, WorldGridParams, apply_grid_key, arrow_delta, destroy_window,
    display_fit_bounds, draw_cam_label, draw_circle_px, draw_debug_lines, draw_help_lines,
    draw_pixel_loupe, draw_rect_px, draw_text_at_px, draw_world_grid, draw_world_track,
    draw_world_velocity, fit_bgr_downscale, hstack_bgr, show_bgr, unscale_xy,
};

/// OpenCV 프리뷰/오버레이 공개 진입점.
pub struct Preview;

impl Preview {
    pub fn fit_bgr_downscale(
        image: &opencv::core::Mat,
        max_w: i32,
        max_h: i32,
    ) -> opencv::Result<FittedBgr> {
        return fit_bgr_downscale(image, max_w, max_h);
    }

    pub fn unscale_xy(x: i32, y: i32, scale: f64) -> (i32, i32) {
        return unscale_xy(x, y, scale);
    }

    pub fn display_fit_bounds() -> Option<(i32, i32)> {
        return display_fit_bounds();
    }

    pub fn show_bgr(
        window: &str,
        image: &opencv::core::Mat,
        wait_ms: i32,
    ) -> opencv::Result<ShowBgrResult> {
        return show_bgr(window, image, wait_ms);
    }

    pub fn destroy_window(window: &str) {
        destroy_window(window);
    }

    pub fn hstack_bgr(panels: &[opencv::core::Mat]) -> opencv::Result<opencv::core::Mat> {
        return hstack_bgr(panels);
    }

    pub fn draw_debug_lines(
        img: &mut opencv::core::Mat,
        lines: &[impl AsRef<str>],
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return draw_debug_lines(img, lines, color);
    }

    pub fn draw_help_lines(
        img: &mut opencv::core::Mat,
        lines: &[impl AsRef<str>],
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return draw_help_lines(img, lines, color);
    }

    pub fn draw_circle_px(
        img: &mut opencv::core::Mat,
        pixel: camera::Pixel,
        radius_px: i32,
        color: opencv::core::Scalar,
        thickness: i32,
    ) -> opencv::Result<()> {
        return draw_circle_px(img, pixel, radius_px, color, thickness);
    }

    pub fn draw_rect_px(
        img: &mut opencv::core::Mat,
        top_left: camera::Pixel,
        width: i32,
        height: i32,
        color: opencv::core::Scalar,
        thickness: i32,
    ) -> opencv::Result<()> {
        return draw_rect_px(img, top_left, width, height, color, thickness);
    }

    pub fn draw_text_at_px(
        img: &mut opencv::core::Mat,
        origin: camera::Pixel,
        text: &str,
        font_scale: f64,
        color: opencv::core::Scalar,
        thickness: i32,
    ) -> opencv::Result<()> {
        return draw_text_at_px(img, origin, text, font_scale, color, thickness);
    }

    pub fn draw_world_velocity(
        img: &mut opencv::core::Mat,
        params: &camera::Params,
        origin: Point3,
        velocity: nalgebra::Vector3<f64>,
        scale_secs: f64,
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return draw_world_velocity(img, params, origin, velocity, scale_secs, color);
    }

    pub fn draw_world_grid(
        img: &mut opencv::core::Mat,
        params: &camera::Params,
        grid: &WorldGridParams,
    ) -> opencv::Result<()> {
        return draw_world_grid(img, params, *grid);
    }

    pub fn draw_world_track(
        img: &mut opencv::core::Mat,
        params: &camera::Params,
        points: &[Point3],
        color: opencv::core::Scalar,
        thickness: i32,
    ) -> opencv::Result<()> {
        return draw_world_track(img, params, points, color, thickness);
    }

    pub fn apply_grid_key(grid: &mut WorldGridParams, key: i32) {
        apply_grid_key(grid, key);
    }

    pub fn draw_cam_label(
        img: &mut opencv::core::Mat,
        label: &str,
        color: opencv::core::Scalar,
    ) -> opencv::Result<()> {
        return draw_cam_label(img, label, color);
    }

    pub fn arrow_delta(key: i32) -> Option<(i32, i32)> {
        return arrow_delta(key);
    }

    pub fn draw_pixel_loupe(
        dst: &mut opencv::core::Mat,
        src: &opencv::core::Mat,
        cx: i32,
        cy: i32,
    ) -> opencv::Result<()> {
        return draw_pixel_loupe(dst, src, cx, cy);
    }
}
