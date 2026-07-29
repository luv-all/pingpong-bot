//! 캘리브 + [`BALL_RADIUS`]로 캠별 Scorer 면적 밴드.

use crate::camera;
use anyhow::{Result, bail};
use nalgebra::Vector3;

use crate::Point3;
use crate::constants::{BALL_RADIUS, table};
use crate::detector::ScorerParams;

const AREA_MIN_SCALE: f64 = 0.5;
const AREA_MAX_SCALE: f64 = 2.5;

/// 테이블 대표점에서 겉보기 공 면적 → `min/max_area_px` (여유 계수 포함).
pub(crate) fn scorer_params_from_calib(
    params: &camera::Params,
    circularity: f64,
) -> Result<ScorerParams> {
    let (min_a, max_a) = ball_area_bounds(params)?;
    let sp = ScorerParams {
        min_area_px: min_a,
        max_area_px: max_a,
        min_circularity: circularity,
    };
    sp.validate()?;
    return Ok(sp);
}

fn ball_area_bounds(params: &camera::Params) -> Result<(f64, f64)> {
    let z = table::SURFACE_Z + BALL_RADIUS;
    let w = table::WIDTH_X;
    let l = table::LENGTH_Y;
    let samples = [
        Point3::new(0.0, 0.0, z),
        Point3::new(w, 0.0, z),
        Point3::new(w, l, z),
        Point3::new(0.0, l, z),
        Point3::new(w * 0.5, l * 0.5, z),
    ];
    let f = (params.fx + params.fy) * 0.5;
    let mut areas = Vec::with_capacity(samples.len());
    for p in samples {
        let x_cam: Vector3<f64> = params.rotation * p.coords + params.translation;
        if x_cam.z <= 0.05 {
            continue;
        }
        let r_px = f * BALL_RADIUS / x_cam.z;
        if !r_px.is_finite() || r_px <= 0.0 {
            continue;
        }
        areas.push(std::f64::consts::PI * r_px * r_px);
    }
    if areas.is_empty() {
        bail!("ball area: no sample points in front of camera");
    }
    let a_min = areas.iter().cloned().fold(f64::INFINITY, f64::min);
    let a_max = areas.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min_area = (a_min * AREA_MIN_SCALE).max(1.0);
    let max_area = (a_max * AREA_MAX_SCALE).max(min_area + 1.0);
    return Ok((min_area, max_area));
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn area_bounds_ordered_and_finite() {
        let params = camera::Params::sim_layout(camera::Id(0), 2);
        let (lo, hi) = ball_area_bounds(&params).unwrap();
        assert!(lo.is_finite() && hi.is_finite());
        assert!(lo > 0.0);
        assert!(hi > lo);
        let sp = scorer_params_from_calib(&params, 0.55).unwrap();
        assert_eq!(sp.min_area_px, lo);
        assert_eq!(sp.max_area_px, hi);
    }

    #[test]
    fn nearer_gives_larger_max_than_defaults_floor() {
        let eye = Vector3::new(table::WIDTH_X * 0.5, -0.5, table::SURFACE_Z + 1.2);
        let target = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.3,
            table::SURFACE_Z,
        );
        let params = camera::Params::look_at(
            camera::Id(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            1280,
            800,
            55.0_f64.to_radians(),
        );
        let (lo, hi) = ball_area_bounds(&params).unwrap();
        // 기본 20..20000보다 훨씬 좁은 밴드여야 함
        assert!(hi < 20_000.0);
        assert!(lo > 20.0 || hi / lo < 50.0);
        let _ = (lo, hi);
    }
}
