//! 월드 테이블 변 투영 → 이미지 직선 → 바닥 제거.
//!
//! 두 캠 모두 테이블 **오른편**(`x > W`)에서 비스듬히 내려보므로 컷은 캠과
//! 무관하게 두 개다 — `x ≥ W+δ`(캠 앞 바닥)와 `y ≥ L+δ`(파 엔드 뒤 바닥)를
//! 버린다.
//!
//! 버릴 영역은 두 반평면의 **합집합**이라 볼록이 아니다. 그래서 반대로 keep
//! 영역 = `이미지 사각형 ∩ 두 반평면`을 잘라 채운다 — 교집합이라 항상 볼록이고,
//! 컷을 늘려도 [`CUTS`]에 한 줄 추가하면 된다.
//!
//! `δ`는 테이블 변에서 캘리브 허용 오차 [`MAX_REPROJ_RMSE_PX`]만큼 **바깥으로**
//! 민 값: `δ = RMSE · Z_cam / fx` [m] — keep가 가장자리·옆면·재투영 오차만큼
//! 넓어진다. 변마다 깊이가 다르므로 `δ`도 변마다 따로 잰다.

use crate::camera;
use anyhow::{Result, bail, ensure};
use opencv::core::{Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use crate::Point3;
use crate::constants::table;
use crate::defaults::MAX_REPROJ_RMSE_PX;

/// 적용 순서대로 자를 컷 — 늘리려면 여기에 추가한다.
const CUTS: [Axis; 2] = [Axis::X, Axis::Y];

/// 버릴 쪽 시험점을 컷 변에서 얼마나 밀어낼지 [m].
const PROBE_OFFSET_M: f64 = 0.05;

/// 테이블 변 하나를 미터로 자르는 축. `좌표 ≥ 변 + δ` 쪽을 버린다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    /// 월드 `x = W + δ` — 캠 앞 바닥.
    X,
    /// 월드 `y = L + δ` — 파 엔드 뒤 바닥.
    Y,
}

impl Axis {
    /// 자를 테이블 변의 월드 좌표 [m].
    fn table_extent(self) -> f64 {
        return match self {
            Self::X => table::WIDTH_X,
            Self::Y => table::LENGTH_Y,
        };
    }

    /// `δ`를 재는 기준점 — 자를 변의 중점.
    fn edge_mid(self) -> Point3 {
        let z = table::SURFACE_Z;
        return match self {
            Self::X => Point3::new(table::WIDTH_X, table::LENGTH_Y * 0.5, z),
            Self::Y => Point3::new(table::WIDTH_X * 0.5, table::LENGTH_Y, z),
        };
    }

    /// 컷 직선의 두 끝점과, 버릴 쪽에 놓인 시험점.
    fn geometry(self, cut: f64) -> (Point3, Point3, Point3) {
        let z = table::SURFACE_Z;
        let probe = cut + PROBE_OFFSET_M;
        return match self {
            Self::X => (
                Point3::new(cut, 0.0, z),
                Point3::new(cut, table::LENGTH_Y, z),
                Point3::new(probe, table::LENGTH_Y * 0.5, z),
            ),
            Self::Y => (
                Point3::new(0.0, cut, z),
                Point3::new(table::WIDTH_X, cut, z),
                Point3::new(table::WIDTH_X * 0.5, probe, z),
            ),
        };
    }

    fn label(self) -> &'static str {
        return match self {
            Self::X => "x",
            Self::Y => "y",
        };
    }
}

/// 컷 변 하나의 투영 결과.
#[derive(Clone, Copy, Debug)]
pub struct CutEdge {
    pub axis: Axis,
    /// 월드 컷 좌표 [m] — `변 + δ`.
    pub cut: f64,
    /// [`MAX_REPROJ_RMSE_PX`] → 미터 환산 마진 [m].
    pub margin_m: f64,
    /// 컷 직선의 투영 양 끝점 `(u, v)`. 비스듬한 옆면 뷰에서는 이 직선이
    /// 이미지에서 거의 수직일 수 있어 row 한 쌍으로는 표현이 안 된다.
    pub p0: (f64, f64),
    pub p1: (f64, f64),
    /// 버릴 쪽의 [`side_of_line`] 부호 (`±1`).
    pub discard_sign: f64,
}

impl CutEdge {
    fn from_params(params: &camera::Params, axis: Axis) -> Result<Self> {
        let Some((_, _, z_cam)) = project_unbounded(params, axis.edge_mid()) else {
            bail!(
                "floor-edge[{}]: table edge midpoint behind camera",
                axis.label()
            );
        };
        let margin_m = MAX_REPROJ_RMSE_PX * z_cam / params.fx;
        ensure!(
            margin_m.is_finite() && margin_m >= 0.0,
            "floor-edge[{}]: bad margin {}",
            axis.label(),
            margin_m
        );

        // keep 여유: 컷을 테이블 바깥으로 민다.
        let cut = axis.table_extent() + margin_m;
        let (a, b, probe) = axis.geometry(cut);
        let Some((u0, v0, _)) = project_unbounded(params, a) else {
            bail!("floor-edge[{}]: cut endpoint behind camera", axis.label());
        };
        let Some((u1, v1, _)) = project_unbounded(params, b) else {
            bail!("floor-edge[{}]: cut endpoint behind camera", axis.label());
        };
        ensure!(
            (u1 - u0).hypot(v1 - v0) > 1e-6,
            "floor-edge[{}]: cut edge projects to a single point",
            axis.label()
        );
        let Some((up, vp, _)) = project_unbounded(params, probe) else {
            bail!(
                "floor-edge[{}]: discard probe not projectable",
                axis.label()
            );
        };

        let signed = side_of_line(u0, v0, u1, v1, up, vp);
        ensure!(
            signed.abs() > 1e-9,
            "floor-edge[{}]: discard probe lies on the cut line",
            axis.label()
        );

        return Ok(Self {
            axis,
            cut,
            margin_m,
            p0: (u0, v0),
            p1: (u1, v1),
            discard_sign: signed.signum(),
        });
    }

    fn line(&self) -> (f64, f64, f64, f64) {
        return (self.p0.0, self.p0.1, self.p1.0, self.p1.1);
    }
}

/// 테이블 변 투영으로 만든 keep 마스크 (255=검출 허용, 0=바닥 제거).
#[derive(Clone)]
pub struct FloorEdgeMask {
    pub keep: Mat,
    /// [`CUTS`] 순서대로의 컷 변.
    pub edges: Vec<CutEdge>,
    /// keep 폴리곤 꼭짓점 수 — 삼각형·사각형·오각형 전부 정상.
    pub keep_poly_len: usize,
    pub width: i32,
    pub height: i32,
}

impl FloorEdgeMask {
    /// 캠 무관 — `x ≥ W+δ`·`y ≥ L+δ` 바닥을 제거한 keep 마스크.
    pub fn from_params(params: &camera::Params) -> Result<Self> {
        let w = params.width as i32;
        let h = params.height as i32;
        ensure!(w > 1 && h > 1, "bad image size {}x{}", w, h);
        ensure!(params.fx > 0.0, "floor-edge: fx must be > 0");

        let edges = CUTS
            .iter()
            .map(|&axis| CutEdge::from_params(params, axis))
            .collect::<Result<Vec<_>>>()?;

        // keep = 이미지 사각형 ∩ (컷마다 버릴 쪽의 반대 반평면). 교집합이라 볼록.
        let mut poly = vec![
            (0.0, 0.0),
            (f64::from(w - 1), 0.0),
            (f64::from(w - 1), f64::from(h - 1)),
            (0.0, f64::from(h - 1)),
        ];
        for edge in &edges {
            poly = clip_to_halfplane(&poly, edge.line(), -edge.discard_sign);
        }
        ensure!(
            poly.len() >= 3,
            "floor-edge: every pixel is cut — check calibration"
        );

        let mut keep = Mat::zeros(h, w, opencv::core::CV_8UC1)?.to_mat()?;
        let vertices = Vector::<Point>::from_iter(
            poly.iter()
                .map(|&(x, y)| Point::new(x.round() as i32, y.round() as i32)),
        );
        imgproc::fill_convex_poly(&mut keep, &vertices, Scalar::all(255.0), imgproc::LINE_8, 0)?;

        return Ok(Self {
            keep,
            keep_poly_len: poly.len(),
            edges,
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

    /// 컷 직선들을 `img`에 그린다. 프레임을 가로지르게 양쪽으로 늘리고
    /// 클리핑은 OpenCV에 맡긴다 (거의 수직인 직선도 그대로 그려진다).
    pub fn draw_edge_lines(&self, img: &mut Mat, color: Scalar, thickness: i32) -> Result<()> {
        let span = f64::from(self.width + self.height);
        for edge in &self.edges {
            let (u0, v0) = edge.p0;
            let (u1, v1) = edge.p1;
            let len = (u1 - u0).hypot(v1 - v0);
            if len < 1e-6 {
                continue;
            }
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
        }
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

    const TEST_W: i32 = 1280;
    const TEST_H: i32 = 800;

    /// 테이블 오른편(`x > W`)에서 비스듬히 내려보는 캠. 해상도·FOV는 실제 리그와
    /// 같게 둔다 — 좁은 FOV로는 테이블이 프레임에 안 들어와 검증이 무의미해진다.
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
            TEST_W as u32,
            TEST_H as u32,
            47.3_f64.to_radians(),
        );
    }

    /// `data/calibration.json` cam0 위치 — 니어 엔드 쪽 오른편.
    fn near_end_cam() -> camera::Params {
        return right_side_oblique(camera::Id(0), Vector3::new(2.87, 0.10, 2.08));
    }

    /// `data/calibration.json` cam1 위치 — 파 엔드 쪽 오른편. 컷 직선 기울기 부호가
    /// [`near_end_cam`]과 반대라, 버릴 쪽을 상수로 박으면 여기서 깨진다.
    fn far_end_cam() -> camera::Params {
        return right_side_oblique(camera::Id(1), Vector3::new(2.94, 2.97, 2.07));
    }

    /// 옆면 정면에서 더 멀리·높이 — keep가 오각형이 아니라 사각형으로 나오는 배치.
    fn side_on_cam() -> camera::Params {
        return right_side_oblique(
            camera::Id(1),
            Vector3::new(
                table::WIDTH_X + 2.0,
                table::LENGTH_Y * 0.5,
                table::SURFACE_Z + 1.5,
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

    fn all_cams() -> [camera::Params; 3] {
        return [near_end_cam(), far_end_cam(), side_on_cam()];
    }

    fn edge(mask: &FloorEdgeMask, axis: Axis) -> CutEdge {
        return *mask
            .edges
            .iter()
            .find(|e| e.axis == axis)
            .expect("axis present");
    }

    #[test]
    fn both_cuts_sit_outside_the_table() {
        for params in all_cams() {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            assert_eq!(mask.edges.len(), 2, "x and y cuts");

            let x = edge(&mask, Axis::X);
            let y = edge(&mask, Axis::Y);
            assert!(x.margin_m > 0.0 && y.margin_m > 0.0, "margins positive");
            assert!(
                x.cut > table::WIDTH_X,
                "x cut must sit outside the +X edge: {}",
                x.cut
            );
            assert!(
                y.cut > table::LENGTH_Y,
                "y cut must sit outside the +Y edge: {}",
                y.cut
            );
            // 변마다 깊이가 달라 δ도 달라야 한다.
            assert_ne!(x.margin_m, y.margin_m, "per-edge margin, not one shared δ");
        }
    }

    /// 테이블 네 모서리와 중앙은 두 컷 모두를 통과해야 한다 — `y=L` 모서리는
    /// 컷 선 위에 정확히 놓이므로 δ가 없으면 잘려나간다.
    #[test]
    fn keeps_whole_table_including_the_far_edge_corners() {
        for params in all_cams() {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            let z = table::SURFACE_Z;
            let probes = [
                (
                    "center",
                    Point3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, z),
                ),
                ("x0 y0", Point3::new(0.0, 0.0, z)),
                ("xW y0", Point3::new(table::WIDTH_X, 0.0, z)),
                ("x0 yL", Point3::new(0.0, table::LENGTH_Y, z)),
                ("xW yL", Point3::new(table::WIDTH_X, table::LENGTH_Y, z)),
            ];
            for (name, p) in probes {
                let (u, v) = project_px(&params, p);
                assert!(
                    (0..TEST_W).contains(&u) && (0..TEST_H).contains(&v),
                    "{name} must be in frame for this test: ({u},{v})"
                );
                assert_eq!(pixel(&mask, u, v), 255, "table {name} should be kept");
            }
        }
    }

    /// 두 컷 바깥 시험점이 모두 프레임에 잡히는 배치로만 검증한다
    /// ([`far_end_cam`]은 `y` 컷 바깥이 화각을 벗어나 확인할 화소가 없다).
    #[test]
    fn removes_floor_beyond_each_cut() {
        for params in [near_end_cam(), side_on_cam()] {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            let z = table::SURFACE_Z;
            let beyond_x = Point3::new(edge(&mask, Axis::X).cut + 0.5, table::LENGTH_Y * 0.5, z);
            let beyond_y = Point3::new(table::WIDTH_X * 0.5, edge(&mask, Axis::Y).cut + 0.5, z);
            for (name, p) in [("x>W+δ", beyond_x), ("y>L+δ", beyond_y)] {
                let (u, v) = project_px(&params, p);
                assert!(
                    (0..TEST_W).contains(&u) && (0..TEST_H).contains(&v),
                    "{name} probe must be in frame for this test: ({u},{v})"
                );
                assert_eq!(pixel(&mask, u, v), 0, "floor at {name} should be removed");
            }
        }
    }

    /// keep 폴리곤 꼭짓점 수는 배치에서 나온다 — 케이스 분기가 없다는 증거.
    /// 리그 두 캠은 오각형, 더 멀리 물러난 옆면 정면은 사각형.
    #[test]
    fn keep_poly_vertex_count_follows_the_geometry() {
        for (name, params, expected) in [
            ("near_end", near_end_cam(), 5),
            ("far_end", far_end_cam(), 5),
            ("side_on", side_on_cam(), 4),
        ] {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            assert!(
                (3..=6).contains(&mask.keep_poly_len),
                "{name}: rect ∩ 2 half-planes has 3..=6 vertices, got {}",
                mask.keep_poly_len
            );
            assert_eq!(mask.keep_poly_len, expected, "{name} keep polygon");
        }
    }

    /// keep은 두 컷 반평면의 교집합과 화소 단위로 일치해야 한다.
    #[test]
    fn keep_matches_the_intersection_of_both_halfplanes() {
        for params in all_cams() {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            for y in (0..TEST_H).step_by(7) {
                for x in (0..TEST_W).step_by(7) {
                    let (fx, fy) = (f64::from(x), f64::from(y));
                    let mut discard = false;
                    let mut near_boundary = false;
                    for e in &mask.edges {
                        let (u0, v0, u1, v1) = e.line();
                        let norm = (u1 - u0).hypot(v1 - v0);
                        let d = e.discard_sign * side_of_line(u0, v0, u1, v1, fx, fy) / norm;
                        if d.abs() < 2.0 {
                            near_boundary = true; // 폴리곤 반올림 오차 구간
                        }
                        if d > 0.0 {
                            discard = true;
                        }
                    }
                    if near_boundary {
                        continue;
                    }
                    let expected = if discard { 0 } else { 255 };
                    assert_eq!(pixel(&mask, x, y), expected, "pixel ({x},{y}) mismatch");
                }
            }
        }
    }

    /// `y ≥ L+δ` 컷이 실제로 화소를 더 지우는지 — x 컷만 걸었을 때와 비교.
    #[test]
    fn y_cut_removes_pixels_the_x_cut_alone_would_keep() {
        for params in all_cams() {
            let mask = FloorEdgeMask::from_params(&params).expect("mask");
            let y_edge = edge(&mask, Axis::Y);
            let x_edge = edge(&mask, Axis::X);

            // x 컷은 통과하지만 y 컷에 걸리는 화소가 프레임 안에 있어야 한다.
            let mut found = false;
            for y in 0..TEST_H {
                for x in 0..TEST_W {
                    let (fx, fy) = (f64::from(x), f64::from(y));
                    let (a0, b0, a1, b1) = x_edge.line();
                    let kept_by_x =
                        x_edge.discard_sign * side_of_line(a0, b0, a1, b1, fx, fy) < 0.0;
                    let (c0, d0, c1, d1) = y_edge.line();
                    let cut_by_y = y_edge.discard_sign * side_of_line(c0, d0, c1, d1, fx, fy) > 0.0;
                    if kept_by_x && cut_by_y && pixel(&mask, x, y) == 0 {
                        found = true;
                        break;
                    }
                }
                if found {
                    break;
                }
            }
            assert!(found, "y cut should remove pixels the x cut alone keeps");
        }
    }

    #[test]
    fn apply_bgr_blacks_masked_pixels() {
        let params = near_end_cam();
        let mask = FloorEdgeMask::from_params(&params).unwrap();
        let bgr = Mat::new_size_with_default(
            opencv::core::Size::new(TEST_W, TEST_H),
            opencv::core::CV_8UC3,
            Scalar::all(200.0),
        )
        .unwrap();

        let mut found = None;
        'outer: for y in 0..TEST_H {
            for x in 0..TEST_W {
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
