//! 탁구대 XY×Z 월드 격자 오버레이 (이 툴 전용).

use opencv::Result as CvResult;
use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use pingpong_bot::constants::table;
use pingpong_bot::{CameraParams, Point3, draw_circle_px, draw_debug_lines};

/// 탁구대 XY×Z 월드 격자 오버레이 파라미터.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldGridParams {
    /// XY 격자 간격 [m]
    pub xy_step: f64,
    /// Z 층 간격 [m] (테이블 면 위)
    pub z_step: f64,
    /// Z 층 수 (≥1). `k = 0..z_layers`, `z = SURFACE_Z + k * z_step`
    pub z_layers: u32,
}

impl Default for WorldGridParams {
    fn default() -> Self {
        return Self {
            xy_step: 0.10,
            z_step: 0.05,
            z_layers: 6,
        };
    }
}

impl WorldGridParams {
    /// 키 조절용 하한 클램프.
    pub fn clamp(self) -> Self {
        return Self {
            xy_step: self.xy_step.max(0.02),
            z_step: self.z_step.max(0.02),
            z_layers: self.z_layers.max(1),
        };
    }
}

/// Z 정규화 t∈[0,1] → BGR jet-like (낮음=빨강 → 높음=파랑/보라).
fn jet_bgr(t: f64) -> Scalar {
    let t = t.clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.25 {
        let u = t / 0.25;
        (1.0, u, 0.0)
    } else if t < 0.5 {
        let u = (t - 0.25) / 0.25;
        (1.0 - u, 1.0, 0.0)
    } else if t < 0.75 {
        let u = (t - 0.5) / 0.25;
        (0.0, 1.0, u)
    } else {
        let u = (t - 0.75) / 0.25;
        (u * 0.5, 1.0 - u, 1.0)
    };
    return Scalar::new(b * 255.0, g * 255.0, r * 255.0, 0.0);
}

fn overlay_scale(img_h: i32) -> f64 {
    return (img_h as f64 / 720.0).clamp(0.5, 1.0);
}

/// 탁구대 XY×Z 격자를 `project_world`로 투영해 그린다.
pub fn draw_world_grid(
    img: &mut Mat,
    params: &CameraParams,
    grid: WorldGridParams,
) -> CvResult<()> {
    let grid = grid.clamp();
    let xy = grid.xy_step;
    let dz = grid.z_step;
    let layers = grid.z_layers;
    let s = overlay_scale(img.rows());
    let radius = (4.0 * s).round().max(2.0) as i32;
    let thickness = -1; // filled

    let mut x = 0.0;
    while x <= table::WIDTH_X + 1e-9 {
        let mut y = 0.0;
        while y <= table::LENGTH_Y + 1e-9 {
            for k in 0..layers {
                let z = table::SURFACE_Z + f64::from(k) * dz;
                let t = if layers <= 1 {
                    0.0
                } else {
                    f64::from(k) / f64::from(layers - 1)
                };
                let color = jet_bgr(t);
                let point = Point3::new(x, y, z);
                if let Some(px) = params.project_world(point) {
                    draw_circle_px(img, px, radius, color, thickness)?;
                }
            }
            y += xy;
        }
        x += xy;
    }

    let lines = [
        "World to Camera".to_string(),
        format!(
            "xy={:.2} z={:.2} layers={}",
            grid.xy_step, grid.z_step, grid.z_layers
        ),
    ];
    draw_debug_lines(img, &lines, Scalar::new(0.0, 0.0, 255.0, 0.0))?;
    return Ok(());
}

pub fn apply_grid_key(grid: &mut WorldGridParams, key: i32) {
    const XY_DELTA: f64 = 0.02;
    const Z_DELTA: f64 = 0.02;
    match key {
        k if k == i32::from(b'=') || k == i32::from(b'+') => {
            grid.xy_step += XY_DELTA;
        }
        k if k == i32::from(b'-') => {
            grid.xy_step -= XY_DELTA;
        }
        k if k == i32::from(b']') => {
            grid.z_layers = grid.z_layers.saturating_add(1);
        }
        k if k == i32::from(b'[') => {
            grid.z_layers = grid.z_layers.saturating_sub(1);
        }
        k if k == i32::from(b'.') => {
            grid.z_step += Z_DELTA;
        }
        k if k == i32::from(b',') => {
            grid.z_step -= Z_DELTA;
        }
        _ => {}
    }
    *grid = grid.clamp();
}
