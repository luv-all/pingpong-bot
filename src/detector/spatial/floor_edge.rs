//! 월드 `x=0`(left) / `x=W`(right) 변 → 이미지 기울어진 직선 → 바닥 사다리꼴 제거.

use anyhow::{Result, bail, ensure};
use opencv::core::{Point, Scalar, Size, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use crate::camera::CameraParams;
use crate::constants::table;
use crate::{CameraId, Point3};

/// 테이블 옆변 투영으로 만든 keep 마스크 (255=검출 허용, 0=바닥 제거).
#[derive(Clone)]
pub struct FloorEdgeMask {
    pub keep: Mat,
    /// 그리기용: 좌·우 경계에서의 직선 row `(f(0), f(W))` [OpenCV y].
    pub line_y_at_left: f64,
    pub line_y_at_right: f64,
    pub width: i32,
    pub height: i32,
}

impl FloorEdgeMask {
    /// `cam_id` 0 → 월드 x=0 변, 그 외 → x=W 변.
    pub fn from_params(cam_id: CameraId, params: &CameraParams) -> Result<Self> {
        let w = params.width as i32;
        let h = params.height as i32;
        ensure!(w > 1 && h > 1, "bad image size {}x{}", w, h);

        let z = table::SURFACE_Z;
        let x_edge = if cam_id.0 == 0 {
            0.0
        } else {
            table::WIDTH_X
        };
        let p0 = Point3::new(x_edge, 0.0, z);
        let p1 = Point3::new(x_edge, table::LENGTH_Y, z);
        let Some((u0, v0, _)) = project_unbounded(params, p0) else {
            bail!("floor-edge: edge endpoint y=0 behind camera");
        };
        let Some((u1, v1, _)) = project_unbounded(params, p1) else {
            bail!("floor-edge: edge endpoint y=L behind camera");
        };

        let y_left = line_y_at_x(u0, v0, u1, v1, 0.0);
        let y_right = line_y_at_x(u0, v0, u1, v1, f64::from(w - 1));

        // 바깥 시험점: left → −X, right → +X
        let eps = 0.05;
        let exterior = if cam_id.0 == 0 {
            Point3::new(-eps, table::LENGTH_Y * 0.5, z)
        } else {
            Point3::new(table::WIDTH_X + eps, table::LENGTH_Y * 0.5, z)
        };
        let Some((ue, ve, _)) = project_unbounded(params, exterior) else {
            bail!("floor-edge: exterior test point not projectable");
        };

        // 하단 사다리꼴 중심이 바깥점과 같은 쪽이면 하단 마스크, 아니면 상단
        let bottom_centroid_y = (y_left + y_right) * 0.5 * 0.5 + f64::from(h - 1) * 0.5;
        let bottom_centroid_x = f64::from(w - 1) * 0.5;
        let exterior_side = side_of_line(u0, v0, u1, v1, ue, ve);
        let bottom_side = side_of_line(u0, v0, u1, v1, bottom_centroid_x, bottom_centroid_y);
        let mask_bottom = exterior_side * bottom_side >= 0.0;

        let mut keep = Mat::new_rows_cols_with_default(
            h,
            w,
            opencv::core::CV_8UC1,
            Scalar::all(255.0),
        )?;

        let yl = y_left.clamp(0.0, f64::from(h - 1));
        let yr = y_right.clamp(0.0, f64::from(h - 1));
        let poly = if mask_bottom {
            Vector::<Point>::from_iter([
                Point::new(0, yl.round() as i32),
                Point::new(0, h - 1),
                Point::new(w - 1, h - 1),
                Point::new(w - 1, yr.round() as i32),
            ])
        } else {
            Vector::<Point>::from_iter([
                Point::new(0, yl.round() as i32),
                Point::new(0, 0),
                Point::new(w - 1, 0),
                Point::new(w - 1, yr.round() as i32),
            ])
        };
        imgproc::fill_convex_poly(
            &mut keep,
            &poly,
            Scalar::all(0.0),
            imgproc::LINE_8,
            0,
        )?;

        return Ok(Self {
            keep,
            line_y_at_left: y_left,
            line_y_at_right: y_right,
            width: w,
            height: h,
        });
    }

    /// keep=0 화소를 검게. 크기가 다르면 에러.
    pub fn apply_bgr(&self, bgr: &Mat) -> Result<Mat> {
        ensure!(
            bgr.cols() == self.width && bgr.rows() == self.height,
            "frame size {}x{} != mask {}x{}",
            bgr.cols(),
            bgr.rows(),
            self.width,
            self.height
        );
        let mut out = Mat::zeros(self.height, self.width, bgr.typ())?.to_mat()?;
        bgr.copy_to_masked(&mut out, &self.keep)?;
        return Ok(out);
    }

    /// 투영 변을 `img`에 그린다 (시안).
    pub fn draw_edge_line(&self, img: &mut Mat, color: Scalar, thickness: i32) -> Result<()> {
        imgproc::line(
            img,
            Point::new(0, self.line_y_at_left.round() as i32),
            Point::new(self.width - 1, self.line_y_at_right.round() as i32),
            color,
            thickness,
            imgproc::LINE_8,
            0,
        )?;
        return Ok(());
    }
}

/// 이미지 경계 무시 핀홀 투영. 카메라 뒤면 None. `(u, v, Z_cam)`.
pub(crate) fn project_unbounded(
    params: &CameraParams,
    point: Point3,
) -> Option<(f64, f64, f64)> {
    let x_cam = params.rotation * point.coords + params.translation;
    if x_cam.z <= 0.05 {
        return None;
    }
    let u = params.fx * (x_cam.x / x_cam.z) + params.cx;
    let v = params.fy * (x_cam.y / x_cam.z) + params.cy;
    return Some((u, v, x_cam.z));
}

fn line_y_at_x(u0: f64, v0: f64, u1: f64, v1: f64, x: f64) -> f64 {
    let dx = u1 - u0;
    if dx.abs() < 1e-9 {
        return (v0 + v1) * 0.5;
    }
    let t = (x - u0) / dx;
    return v0 + t * (v1 - v0);
}

/// >0 / <0 / 0 = 직선 한쪽.
fn side_of_line(u0: f64, v0: f64, u1: f64, v1: f64, qx: f64, qy: f64) -> f64 {
    return (u1 - u0) * (qy - v0) - (v1 - v0) * (qx - u0);
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    fn overhead_looking_down() -> CameraParams {
        // 테이블 중앙 위쪽 — x=0 변이 이미지에 기울어져 보임
        let eye = Vector3::new(-0.4, table::LENGTH_Y * 0.5, table::SURFACE_Z + 1.6);
        let target = Vector3::new(
            table::WIDTH_X * 0.35,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z,
        );
        return CameraParams::look_at(
            CameraId(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            55.0_f64.to_radians(),
        );
    }

    #[test]
    fn floor_mask_zeros_exterior_keeps_table_center() {
        let params = overhead_looking_down();
        let mask = FloorEdgeMask::from_params(CameraId(0), &params).expect("mask");
        assert_eq!(mask.keep.cols(), 640);
        assert_eq!(mask.keep.rows(), 480);

        let center = Point3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z,
        );
        let exterior = Point3::new(-0.3, table::LENGTH_Y * 0.5, table::SURFACE_Z);

        let (uc, vc, _) = project_unbounded(&params, center).unwrap();
        let (ue, ve, _) = project_unbounded(&params, exterior).unwrap();
        let uc = uc.round() as i32;
        let vc = vc.round() as i32;
        let ue = ue.round() as i32;
        let ve = ve.round() as i32;

        if (0..640).contains(&uc) && (0..480).contains(&vc) {
            let k: u8 = *mask.keep.at_2d(vc, uc).unwrap();
            assert_eq!(k, 255, "table center should be kept");
        }
        if (0..640).contains(&ue) && (0..480).contains(&ve) {
            let k: u8 = *mask.keep.at_2d(ve, ue).unwrap();
            assert_eq!(k, 0, "exterior -X should be masked");
        }
    }

    #[test]
    fn apply_bgr_blacks_masked_pixels() {
        let params = overhead_looking_down();
        let mask = FloorEdgeMask::from_params(CameraId(0), &params).unwrap();
        let mut bgr =
            Mat::new_size_with_default(Size::new(640, 480), opencv::core::CV_8UC3, Scalar::all(200.0))
                .unwrap();
        // force one masked pixel known: find any keep==0
        let mut found = None;
        'outer: for y in 0..480 {
            for x in 0..640 {
                let k: u8 = *mask.keep.at_2d(y, x).unwrap();
                if k == 0 {
                    found = Some((x, y));
                    break 'outer;
                }
            }
        }
        let (x, y) = found.expect("mask should remove some pixels");
        let out = mask.apply_bgr(&bgr).unwrap();
        let px: opencv::core::Vec3b = *out.at_2d(y, x).unwrap();
        assert_eq!(px[0], 0);
        assert_eq!(px[1], 0);
        assert_eq!(px[2], 0);
        let _ = &mut bgr;
    }
}
