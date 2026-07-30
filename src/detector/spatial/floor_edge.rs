//! 월드 옆변 투영 → 이미지 기울어진 직선 → 바닥 사다리꼴 제거.
//!
//! 두 캠 모두 테이블 **오른편**(`x > W`)에서 비스듬히 내려보므로, 컷 변은
//! 캠과 무관하게 `x = W + δ` 하나다 — `x ≥ W+δ` 쪽(캠 앞 바닥)을 제거한다.
//! `δ`는 테이블 변(`x=W`)이 아니라 캘리브 허용 오차 [`MAX_REPROJ_RMSE_PX`]만큼
//! **바깥으로** 민 값: `δ = RMSE · Z_cam / fx` [m] — keep가 가장자리·옆면·
//! 재투영 오차만큼 넓어진다.

use crate::camera;
use anyhow::{Result, bail, ensure};
use opencv::core::{Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use crate::Point3;
use crate::constants::table;
use crate::defaults::MAX_REPROJ_RMSE_PX;

/// 테이블 옆변 투영으로 만든 keep 마스크 (255=검출 허용, 0=바닥 제거).
#[derive(Clone)]
pub struct FloorEdgeMask {
    pub keep: Mat,
    /// 그리기용: 컷 변의 투영 양 끝점 `(u, v)` — 월드 `y=0` / `y=L`.
    /// 비스듬한 옆면 뷰에서는 이 직선이 이미지에서 거의 수직일 수 있어,
    /// row 한 쌍(`f(0)`, `f(W)`)으로는 표현이 안 된다.
    pub edge_p0: (f64, f64),
    pub edge_p1: (f64, f64),
    /// 월드 컷 변의 x [m] (`W+δ`).
    pub cut_x: f64,
    /// [`MAX_REPROJ_RMSE_PX`] → 미터 환산 마진 [m].
    pub margin_m: f64,
    /// 제거된 바닥 폴리곤 꼭짓점 수 — 0(제거 없음)·3(삼각형)·4·5 모두 정상.
    pub cut_poly_len: usize,
    pub width: i32,
    pub height: i32,
}

impl FloorEdgeMask {
    /// 캠 무관 — 월드 `x=W+δ` 변으로 자르고 `x ≥ W+δ` 쪽 바닥을 제거한다.
    pub fn from_params(params: &camera::Params) -> Result<Self> {
        let w = params.width as i32;
        let h = params.height as i32;
        ensure!(w > 1 && h > 1, "bad image size {}x{}", w, h);

        let z = table::SURFACE_Z;
        let edge_mid = Point3::new(table::WIDTH_X, table::LENGTH_Y * 0.5, z);
        let Some((_, _, z_cam)) = project_unbounded(params, edge_mid) else {
            bail!("floor-edge: table edge midpoint behind camera");
        };
        ensure!(params.fx > 0.0, "floor-edge: fx must be > 0");
        let margin_m = MAX_REPROJ_RMSE_PX * z_cam / params.fx;
        ensure!(
            margin_m.is_finite() && margin_m >= 0.0,
            "floor-edge: bad margin"
        );

        // keep 여유: 컷을 바닥 쪽(캠 쪽)으로 민다.
        let cut_x = table::WIDTH_X + margin_m;
        let p0 = Point3::new(cut_x, 0.0, z);
        let p1 = Point3::new(cut_x, table::LENGTH_Y, z);
        let Some((u0, v0, _)) = project_unbounded(params, p0) else {
            bail!("floor-edge: edge endpoint y=0 behind camera");
        };
        let Some((u1, v1, _)) = project_unbounded(params, p1) else {
            bail!("floor-edge: edge endpoint y=L behind camera");
        };

        ensure!(
            (u1 - u0).hypot(v1 - v0) > 1e-6,
            "floor-edge: cut edge projects to a single point"
        );

        // 바깥 시험점: cut보다 더 +X 쪽
        let eps = 0.05;
        let exterior = Point3::new(cut_x + eps, table::LENGTH_Y * 0.5, z);
        let Some((ue, ve, _)) = project_unbounded(params, exterior) else {
            bail!("floor-edge: exterior test point not projectable");
        };
        let exterior_side = side_of_line(u0, v0, u1, v1, ue, ve);
        ensure!(
            exterior_side.abs() > 1e-9,
            "floor-edge: exterior test point lies on the cut line"
        );

        // 이미지 사각형 ∩ 바깥 반평면 — 결과는 볼록이고, 컷 직선이 프레임 안에서
        // 위/아래 변으로 빠져나가면 사각형이 아니라 삼각형(또는 오각형)이 된다.
        let line = (u0, v0, u1, v1);
        let rect = [
            (0.0, 0.0),
            (f64::from(w - 1), 0.0),
            (f64::from(w - 1), f64::from(h - 1)),
            (0.0, f64::from(h - 1)),
        ];
        let cut_poly = clip_to_halfplane(&rect, line, exterior_side.signum());

        let mut keep =
            Mat::new_rows_cols_with_default(h, w, opencv::core::CV_8UC1, Scalar::all(255.0))?;
        if cut_poly.len() >= 3 {
            let poly = Vector::<Point>::from_iter(
                cut_poly
                    .iter()
                    .map(|&(x, y)| Point::new(x.round() as i32, y.round() as i32)),
            );
            imgproc::fill_convex_poly(&mut keep, &poly, Scalar::all(0.0), imgproc::LINE_8, 0)?;
        }

        return Ok(Self {
            keep,
            edge_p0: (u0, v0),
            edge_p1: (u1, v1),
            cut_x,
            margin_m,
            cut_poly_len: cut_poly.len(),
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

    /// 투영 컷 변을 `img`에 그린다. 프레임을 가로지르게 양쪽으로 늘리고
    /// 클리핑은 OpenCV에 맡긴다 (거의 수직인 직선도 그대로 그려진다).
    pub fn draw_edge_line(&self, img: &mut Mat, color: Scalar, thickness: i32) -> Result<()> {
        let (u0, v0) = self.edge_p0;
        let (u1, v1) = self.edge_p1;
        let len = (u1 - u0).hypot(v1 - v0);
        if len < 1e-6 {
            return Ok(());
        }
        let span = f64::from(self.width + self.height);
        let (dx, dy) = ((u1 - u0) / len * span, (v1 - v0) / len * span);
        imgproc::line(
            img,
            Point::new((u0 - dx).round() as i32, (v0 - dy).round() as i32),
            Point::new((u1 + dx).round() as i32, (v1 + dy).round() as i32),
            color,
            thickness,
            imgproc::LINE_8,
            0,
        )?;
        return Ok(());
    }
}

/// 이미지 경계 무시 핀홀 투영. 카메라 뒤면 None. `(u, v, Z_cam)`.
pub(crate) fn project_unbounded(params: &camera::Params, point: Point3) -> Option<(f64, f64, f64)> {
    let x_cam = params.rotation * point.coords + params.translation;
    if x_cam.z <= 0.05 {
        return None;
    }
    let u = params.fx * (x_cam.x / x_cam.z) + params.cx;
    let v = params.fy * (x_cam.y / x_cam.z) + params.cy;
    return Some((u, v, x_cam.z));
}

/// >0 / <0 / 0 = 직선 한쪽.
fn side_of_line(u0: f64, v0: f64, u1: f64, v1: f64, qx: f64, qy: f64) -> f64 {
    return (u1 - u0) * (qy - v0) - (v1 - v0) * (qx - u0);
}

/// 볼록 폴리곤을 `sign · side_of_line ≥ 0` 반평면으로 자른다 (Sutherland–Hodgman).
/// 사각형 입력이면 결과는 빈 폴리곤·삼각형·사각형·오각형 중 하나.
fn clip_to_halfplane(
    poly: &[(f64, f64)],
    line: (f64, f64, f64, f64),
    sign: f64,
) -> Vec<(f64, f64)> {
    let (u0, v0, u1, v1) = line;
    let dist = |p: (f64, f64)| sign * side_of_line(u0, v0, u1, v1, p.0, p.1);

    let mut out = Vec::with_capacity(poly.len() + 1);
    for i in 0..poly.len() {
        let cur = poly[i];
        let next = poly[(i + 1) % poly.len()];
        let (dc, dn) = (dist(cur), dist(next));
        if dc >= 0.0 {
            out.push(cur);
        }
        if (dc > 0.0 && dn < 0.0) || (dc < 0.0 && dn > 0.0) {
            let t = dc / (dc - dn);
            out.push((cur.0 + t * (next.0 - cur.0), cur.1 + t * (next.1 - cur.1)));
        }
    }
    return out;
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    /// 테이블 오른편(`x > W`)에서 비스듬히 내려보는 캠 — 실제 리그 배치.
    fn right_side_oblique(cam_id: camera::Id, eye: Vector3<f64>) -> camera::Params {
        let target = Vector3::new(
            table::WIDTH_X * 0.5,
            table::LENGTH_Y * 0.5,
            table::SURFACE_Z,
        );
        return camera::Params::look_at(
            cam_id,
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            55.0_f64.to_radians(),
        );
    }

    /// 니어 엔드 쪽에서 비스듬히 — 컷 직선이 프레임 안에서 아래 변으로 빠진다.
    fn near_end_cam() -> camera::Params {
        return right_side_oblique(
            camera::Id(0),
            Vector3::new(table::WIDTH_X + 1.3, -0.6, table::SURFACE_Z + 1.1),
        );
    }

    /// 파 엔드 쪽에서 비스듬히 — 기울기 부호가 반대.
    fn far_end_cam() -> camera::Params {
        return right_side_oblique(
            camera::Id(1),
            Vector3::new(table::WIDTH_X + 1.5, 3.4, table::SURFACE_Z + 1.3),
        );
    }

    /// 옆면 정면 — 컷 직선이 좌우 변을 가로지르는 기존 사다리꼴 케이스.
    fn side_on_cam() -> camera::Params {
        return right_side_oblique(
            camera::Id(1),
            Vector3::new(
                table::WIDTH_X + 1.6,
                table::LENGTH_Y * 0.5,
                table::SURFACE_Z + 1.2,
            ),
        );
    }

    fn pixel(mask: &FloorEdgeMask, x: i32, y: i32) -> u8 {
        return *mask.keep.at_2d(y, x).unwrap();
    }

    fn project_px(params: &camera::Params, p: Point3) -> (i32, i32) {
        let (u, v, _) = project_unbounded(params, p).expect("projectable");
        return (u.round() as i32, v.round() as i32);
    }

    #[test]
    fn both_cams_cut_on_the_right_side() {
        for params in [near_end_cam(), far_end_cam(), side_on_cam()] {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            assert!(mask.margin_m > 0.0, "rmse margin should be positive");
            assert!(
                mask.cut_x > table::WIDTH_X,
                "cut must sit outside the +X edge: {}",
                mask.cut_x
            );
        }
    }

    #[test]
    fn keeps_table_center_and_removes_plus_x_floor() {
        for params in [near_end_cam(), far_end_cam(), side_on_cam()] {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            let center = Point3::new(
                table::WIDTH_X * 0.5,
                table::LENGTH_Y * 0.5,
                table::SURFACE_Z,
            );
            let exterior = Point3::new(
                table::WIDTH_X + 0.6,
                table::LENGTH_Y * 0.5,
                table::SURFACE_Z,
            );

            let (uc, vc) = project_px(&params, center);
            let (ue, ve) = project_px(&params, exterior);
            assert!(
                (0..640).contains(&uc) && (0..480).contains(&vc),
                "table center must be in frame for this test: ({uc},{vc})"
            );
            assert!(
                (0..640).contains(&ue) && (0..480).contains(&ve),
                "exterior probe must be in frame for this test: ({ue},{ve})"
            );
            assert_eq!(pixel(&mask, uc, vc), 255, "table center should be kept");
            assert_eq!(pixel(&mask, ue, ve), 0, "floor at x>W+δ should be removed");
        }
    }

    /// 비스듬한 뷰에서는 컷 폴리곤이 사각형이 아니라 삼각형이 된다 —
    /// `(0,f(0))`·`(W,f(W))` 사다리꼴로 고정하면 이 케이스에서 과잉 마스킹된다.
    #[test]
    fn oblique_view_cuts_a_triangle_not_a_quad() {
        let params = near_end_cam();
        let mask = FloorEdgeMask::from_params(&params).expect("mask");
        assert_eq!(mask.cut_poly_len, 3, "oblique cut should be a triangle");
        // 사다리꼴 가정이 잘못 지웠던 좌하단 모서리가 살아 있어야 한다.
        assert_eq!(
            pixel(&mask, 0, 479),
            255,
            "bottom-left corner is on the keep side of the cut"
        );
    }

    #[test]
    fn side_on_view_still_cuts_a_quad() {
        let mask = FloorEdgeMask::from_params(&side_on_cam()).expect("mask");
        assert_eq!(mask.cut_poly_len, 4, "side-on cut spans both image borders");
    }

    /// keep은 컷 직선의 반평면과 화소 단위로 일치해야 한다 (폴리곤 모양 무관).
    #[test]
    fn keep_matches_the_cut_halfplane() {
        for params in [near_end_cam(), far_end_cam(), side_on_cam()] {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            let (u0, v0) = mask.edge_p0;
            let (u1, v1) = mask.edge_p1;
            let exterior = Point3::new(mask.cut_x + 0.05, table::LENGTH_Y * 0.5, table::SURFACE_Z);
            let (ue, ve, _) = project_unbounded(&params, exterior).unwrap();
            let sign = side_of_line(u0, v0, u1, v1, ue, ve).signum();
            let norm = (u1 - u0).hypot(v1 - v0);

            for y in (0..480).step_by(7) {
                for x in (0..640).step_by(7) {
                    let d = sign * side_of_line(u0, v0, u1, v1, f64::from(x), f64::from(y)) / norm;
                    if d.abs() < 2.0 {
                        continue; // 경계 ±2px는 폴리곤 반올림 오차 구간
                    }
                    let expected = if d > 0.0 { 0 } else { 255 };
                    assert_eq!(
                        pixel(&mask, x, y),
                        expected,
                        "pixel ({x},{y}) d={d:.2} mismatch"
                    );
                }
            }
        }
    }

    #[test]
    fn apply_bgr_blacks_masked_pixels() {
        let params = near_end_cam();
        let mask = FloorEdgeMask::from_params(&params).unwrap();
        let bgr = Mat::new_size_with_default(
            opencv::core::Size::new(640, 480),
            opencv::core::CV_8UC3,
            Scalar::all(200.0),
        )
        .unwrap();

        let mut found = None;
        'outer: for y in 0..480 {
            for x in 0..640 {
                if pixel(&mask, x, y) == 0 {
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
    }

    #[test]
    fn size_mismatch_is_an_error() {
        let mask = FloorEdgeMask::from_params(&near_end_cam()).unwrap();
        let small = Mat::new_size_with_default(
            opencv::core::Size::new(320, 240),
            opencv::core::CV_8UC3,
            Scalar::all(10.0),
        )
        .unwrap();
        assert!(mask.apply_bgr(&small).is_err());
    }
}
