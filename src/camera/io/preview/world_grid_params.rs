use opencv::Result as CvResult;
use opencv::core::{Mat, Point, Scalar};
use opencv::imgproc;
use opencv::prelude::*;

use crate::Point3;
use crate::camera;

use super::ops::{draw_circle_px, overlay_scale};

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

fn project_grid_pt(params: &camera::Params, x: f64, y: f64, z: f64) -> Option<Point> {
    let px = params.project_world(Point3::new(x, y, z))?;
    return Some(Point::new(px.x.round() as i32, px.y.round() as i32));
}

/// `[0, max]` 축 샘플. `step`으로 채우되 끝점 `max`는 항상 포함 (탁구대 네 변 보장).
fn grid_axis_inclusive(max: f64, step: f64) -> Vec<f64> {
    debug_assert!(max >= 0.0 && step > 0.0);
    let mut v = Vec::new();
    let mut t = 0.0;
    while t < max - 1e-9 {
        v.push(t);
        t += step;
    }
    match v.last() {
        Some(&last) if (max - last).abs() <= 1e-9 => {}
        _ => v.push(max),
    }
    return v;
}

/// 탁구대 XY×Z 격자를 `project_world`로 투영해 점+선분으로 그린다.
/// XY는 간격과 무관하게 `0`·`WIDTH_X`·`LENGTH_Y` 경계를 항상 포함한다.
pub fn draw_world_grid(
    img: &mut Mat,
    params: &camera::Params,
    grid: WorldGridParams,
) -> CvResult<()> {
    use crate::constants::table;

    let grid = grid.clamp();
    let xy = grid.xy_step;
    let dz = grid.z_step;
    let layers = grid.z_layers;
    let s = overlay_scale(img.rows());
    let radius = (4.0 * s).round().max(2.0) as i32;
    let line_th = (1.0 * s).round().max(1.0) as i32;

    let xs = grid_axis_inclusive(table::WIDTH_X, xy);
    let ys = grid_axis_inclusive(table::LENGTH_Y, xy);

    for (ki, k) in (0..layers).enumerate() {
        let z = table::SURFACE_Z + f64::from(k) * dz;
        let t = if layers <= 1 {
            0.0
        } else {
            f64::from(k) / f64::from(layers - 1)
        };
        let color = jet_bgr(t);

        for (i, &x) in xs.iter().enumerate() {
            for (j, &y) in ys.iter().enumerate() {
                let Some(p0) = project_grid_pt(params, x, y, z) else {
                    continue;
                };
                if i + 1 < xs.len() {
                    if let Some(p1) = project_grid_pt(params, xs[i + 1], y, z) {
                        imgproc::line(img, p0, p1, color, line_th, imgproc::LINE_AA, 0)?;
                    }
                }
                if j + 1 < ys.len() {
                    if let Some(p1) = project_grid_pt(params, x, ys[j + 1], z) {
                        imgproc::line(img, p0, p1, color, line_th, imgproc::LINE_AA, 0)?;
                    }
                }
            }
        }

        if ki + 1 < layers as usize {
            let z2 = table::SURFACE_Z + f64::from(k + 1) * dz;
            for &x in &xs {
                for &y in &ys {
                    let Some(p0) = project_grid_pt(params, x, y, z) else {
                        continue;
                    };
                    if let Some(p1) = project_grid_pt(params, x, y, z2) {
                        imgproc::line(img, p0, p1, color, line_th, imgproc::LINE_AA, 0)?;
                    }
                }
            }
        }
    }

    for &x in &xs {
        for &y in &ys {
            for k in 0..layers {
                let z = table::SURFACE_Z + f64::from(k) * dz;
                let t = if layers <= 1 {
                    0.0
                } else {
                    f64::from(k) / f64::from(layers - 1)
                };
                let color = jet_bgr(t);
                if let Some(px) = params.project_world(Point3::new(x, y, z)) {
                    draw_circle_px(img, px, radius, color, -1)?;
                }
            }
        }
    }

    // HUD는 그리지 않음 — 호출측 `draw_debug_lines`와 좌상단이 겹친다.
    return Ok(());
}

/// 격자 키: `+/-` XY, `[]` layers, `.,` Z step.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_axis_always_includes_table_edges() {
        use crate::constants::table;
        let xs = grid_axis_inclusive(table::WIDTH_X, 0.10);
        let ys = grid_axis_inclusive(table::LENGTH_Y, 0.10);
        assert!((xs[0] - 0.0).abs() < 1e-12);
        assert!((ys[0] - 0.0).abs() < 1e-12);
        assert!((xs.last().copied().unwrap() - table::WIDTH_X).abs() < 1e-12);
        assert!((ys.last().copied().unwrap() - table::LENGTH_Y).abs() < 1e-12);
        // step이 max를 나눠 떨어질 때 끝점 중복 없음
        let even = grid_axis_inclusive(1.0, 0.25);
        assert_eq!(even, vec![0.0, 0.25, 0.5, 0.75, 1.0]);
    }
}
