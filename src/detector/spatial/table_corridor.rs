//! 테이블 XY 프리즘(상판 + 비행 밴드) 투영 → convex hull keep 마스크.
//!
//! floor-edge가 "바닥을 지우는" 마스크라면, 이쪽은 "대 위만 남기는" 마스크다.
//! XY는 [`MAX_REPROJ_RMSE_PX`] → 미터 환산 마진만큼 바깥으로 팽창한다.

use crate::camera;
use anyhow::{Result, bail, ensure};
use opencv::core::{Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::floor_edge::project_unbounded;
use crate::Point3;
use crate::constants::table;
use crate::defaults::MAX_REPROJ_RMSE_PX;

/// 테이블 복도 keep 마스크 (255=검출 허용, 0=복도 밖).
#[derive(Clone)]
pub struct TableCorridorMask {
    pub keep: Mat,
    /// 그리기용 convex hull (이미지 좌표).
    pub hull: Vector<Point>,
    /// 상판 위 비행 밴드 높이 [m].
    pub band_m: f64,
    /// XY 팽창 마진 [m].
    pub margin_m: f64,
    pub width: i32,
    pub height: i32,
}

impl TableCorridorMask {
    /// 상판 사각형을 `margin_m` 팽창 + `band_m` 높이 프리즘 → 8꼭짓점 투영 → hull fill.
    pub fn from_params(params: &camera::Params, band_m: f64) -> Result<Self> {
        let w = params.width as i32;
        let h = params.height as i32;
        ensure!(w > 1 && h > 1, "corridor: bad image size {}x{}", w, h);
        ensure!(params.fx > 0.0, "corridor: fx must be > 0");
        ensure!(band_m > 0.0, "corridor: band_m must be > 0");

        let z0 = table::SURFACE_Z;
        let center = Point3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, z0);
        let Some((_, _, z_cam)) = project_unbounded(params, center) else {
            bail!("corridor: table center behind camera");
        };
        let margin_m = MAX_REPROJ_RMSE_PX * z_cam / params.fx;
        ensure!(
            margin_m.is_finite() && margin_m >= 0.0,
            "corridor: bad margin"
        );

        let x0 = -margin_m;
        let x1 = table::WIDTH_X + margin_m;
        let y0 = -margin_m;
        let y1 = table::LENGTH_Y + margin_m;
        let z1 = z0 + band_m;

        let mut pts = Vector::<Point>::new();
        for z in [z0, z1] {
            for (x, y) in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)] {
                let Some((u, v, _)) = project_unbounded(params, Point3::new(x, y, z)) else {
                    continue;
                };
                // 프레임 밖 꼭짓점도 hull에 넣되, 좌표는 넉넉히 clamp해 오버플로를 막는다.
                let u = u.clamp(f64::from(-w), f64::from(2 * w));
                let v = v.clamp(f64::from(-h), f64::from(2 * h));
                pts.push(Point::new(u.round() as i32, v.round() as i32));
            }
        }
        ensure!(
            pts.len() >= 3,
            "corridor: too few projectable corners ({})",
            pts.len()
        );

        let mut hull = Vector::<Point>::new();
        imgproc::convex_hull(&pts, &mut hull, true, true)?;

        let mut keep =
            Mat::new_rows_cols_with_default(h, w, opencv::core::CV_8UC1, Scalar::all(0.0))?;
        imgproc::fill_convex_poly(&mut keep, &hull, Scalar::all(255.0), imgproc::LINE_8, 0)?;

        return Ok(Self {
            keep,
            hull,
            band_m,
            margin_m,
            width: w,
            height: h,
        });
    }

    /// keep hull 외곽선을 `img`에 그린다.
    pub fn draw_hull(&self, img: &mut Mat, color: Scalar, thickness: i32) -> Result<()> {
        let polys = Vector::<Vector<Point>>::from_iter([self.hull.clone()]);
        imgproc::polylines(img, &polys, true, color, thickness, imgproc::LINE_8, 0)?;
        return Ok(());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    /// 실제 리그와 같은 계열 — 테이블 옆에서 상판을 가로질러 본다.
    fn side_looking_across() -> camera::Params {
        let eye = Vector3::new(-0.8, table::LENGTH_Y * 0.5, table::SURFACE_Z + 0.6);
        let target = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z + 0.2,
        );
        return camera::Params::look_at(
            camera::Id(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            55.0_f64.to_radians(),
        );
    }

    fn keep_at(mask: &TableCorridorMask, params: &camera::Params, p: Point3) -> Option<u8> {
        let (u, v, _) = project_unbounded(params, p)?;
        let (u, v) = (u.round() as i32, v.round() as i32);
        if !(0..mask.width).contains(&u) || !(0..mask.height).contains(&v) {
            return None;
        }
        return mask.keep.at_2d::<u8>(v, u).ok().copied();
    }

    #[test]
    fn corridor_keeps_table_and_band_drops_far_exterior() {
        let params = side_looking_across();
        let mask = TableCorridorMask::from_params(&params, 1.0).expect("corridor");
        assert_eq!(mask.keep.cols(), 640);
        assert_eq!(mask.keep.rows(), 480);
        assert!(mask.margin_m > 0.0, "rmse margin should be positive");

        let center = Point3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z,
        );
        let above = Point3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z + 0.5,
        );
        // 밴드 위 배경(모니터·벽 높이) — 복도가 잘라야 하는 대표 오탐 위치.
        let over_band = Point3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z + 1.0 + 1.5,
        );

        let center_keep = keep_at(&mask, &params, center).expect("table center must be in frame");
        assert_eq!(center_keep, 255, "table surface must be kept");
        let above_keep = keep_at(&mask, &params, above).expect("in-band point must be in frame");
        assert_eq!(above_keep, 255, "in-band point must be kept");
        // 프레임 밖으로 나갈 수 있으므로 hull 포함 관계로 본다.
        // 복도는 시선 원뿔이라 "프리즘 뒤"는 못 자르지만, 밴드 **위**는 반드시 잘라야 한다.
        let (bu, bv, _) = project_unbounded(&params, over_band).expect("over-band point projects");
        let signed = imgproc::point_polygon_test(
            &mask.hull,
            opencv::core::Point2f::new(bu as f32, bv as f32),
            false,
        )
        .unwrap();
        assert!(
            signed < 0.0,
            "point above flight band must be cut: {signed}"
        );

        // keep가 프레임 전체가 되어 검사가 무의미해지는 일이 없어야 한다.
        let kept = opencv::core::count_non_zero(&mask.keep).unwrap();
        assert!(kept > 0, "corridor kept nothing");
        assert!(kept < 640 * 480, "corridor kept the whole frame");
    }

    #[test]
    fn band_height_widens_keep_area() {
        let params = side_looking_across();
        let low = TableCorridorMask::from_params(&params, 0.2).unwrap();
        let high = TableCorridorMask::from_params(&params, 1.5).unwrap();
        let low_n = opencv::core::count_non_zero(&low.keep).unwrap();
        let high_n = opencv::core::count_non_zero(&high.keep).unwrap();
        assert!(
            high_n > low_n,
            "taller band should keep more: {high_n} vs {low_n}"
        );
    }
}
