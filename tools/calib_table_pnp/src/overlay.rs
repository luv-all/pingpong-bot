//! Review 캔버스 그리기 — 패딩·클릭·재투영·잔차.
//!
//! `clicks`는 **이미지 좌표**(`--pad N`이면 음수·폭 초과 가능)이고 캔버스는
//! `(pad, pad)`만큼 밀려 있다. 좌표 변환은 [`to_canvas`] 하나로만 한다.

use anyhow::Result;
use opencv::core::{Mat, Point, Rect, Scalar, Vec3b};
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::camera::Landmark;
use pingpong_bot::camera::TablePnp;
use pingpong_bot::constants::TABLE_LANDMARK_COUNT;

const GREEN: Scalar = Scalar::new(0.0, 255.0, 0.0, 0.0);
const ORANGE: Scalar = Scalar::new(255.0, 128.0, 0.0, 0.0);
const MAGENTA: Scalar = Scalar::new(255.0, 0.0, 255.0, 0.0);
const YELLOW: Scalar = Scalar::new(0.0, 255.0, 255.0, 0.0);
/// 선택된 점 — 초록 클릭점과 확실히 구분되는 시안.
const CYAN: Scalar = Scalar::new(255.0, 255.0, 0.0, 0.0);
/// anchor 마커·refine 반경 원.
const GRAY: Scalar = Scalar::new(160.0, 160.0, 160.0, 0.0);

pub fn to_canvas(p: camera::Pixel, pad: i32) -> camera::Pixel {
    return camera::Pixel::new(p.x + f64::from(pad), p.y + f64::from(pad));
}

pub fn to_canvas_pts(pts: &[camera::Pixel], pad: i32) -> Vec<camera::Pixel> {
    return pts.iter().copied().map(|p| to_canvas(p, pad)).collect();
}

fn pt(p: camera::Pixel) -> Point {
    return Point::new(p.x.round() as i32, p.y.round() as i32);
}

/// Review용: 회색 체크 패딩 + 프레임. `pad==0`이면 프레임 복제.
pub fn make_padded_canvas(frame: &Mat, pad: i32) -> Result<Mat> {
    if pad <= 0 {
        return frame.try_clone().map_err(|e| anyhow::anyhow!("clone: {e}"));
    }
    let fw = frame.cols();
    let fh = frame.rows();
    let cw = fw + 2 * pad;
    let ch = fh + 2 * pad;
    let mut out = Mat::zeros(ch, cw, frame.typ())?.to_mat()?;
    for y in 0..ch {
        for x in 0..cw {
            let g = if (x + y) % 2 == 0 { 40u8 } else { 72u8 };
            *out.at_2d_mut::<Vec3b>(y, x)? = Vec3b::from([g, g, g]);
        }
    }
    {
        let roi = Rect::new(pad, pad, fw, fh);
        let mut dst = Mat::roi_mut(&mut out, roi)?;
        frame.copy_to(&mut dst)?;
    }
    return Ok(out);
}

fn draw_complete_edges(
    panel: &mut Mat,
    pts: &[camera::Pixel],
    color: Scalar,
    thickness: i32,
) -> Result<()> {
    for i in 0..pts.len() {
        for j in (i + 1)..pts.len() {
            imgproc::line(
                panel,
                pt(pts[i]),
                pt(pts[j]),
                color,
                thickness,
                imgproc::LINE_AA,
                0,
            )?;
        }
    }
    return Ok(());
}

fn draw_mesh_edges(
    panel: &mut Mat,
    pts: &[camera::Pixel],
    color: Scalar,
    thickness: i32,
) -> Result<()> {
    for &(a_i, b_i) in TablePnp::landmark_mesh_edges() {
        if a_i >= pts.len() || b_i >= pts.len() {
            continue;
        }
        imgproc::line(
            panel,
            pt(pts[a_i]),
            pt(pts[b_i]),
            color,
            thickness,
            imgproc::LINE_AA,
            0,
        )?;
    }
    return Ok(());
}

/// 클릭 점(녹색) + 현재 꼭짓점 완전연결 메시(주황). `sel`은 시안으로 강조.
pub fn draw_clicks(
    panel: &mut Mat,
    clicks: &[camera::Pixel],
    marks: &[Landmark],
    pad: i32,
    sel: Option<usize>,
) -> Result<()> {
    let pts = to_canvas_pts(clicks, pad);
    draw_complete_edges(panel, &pts, ORANGE, 1)?;

    for (i, px) in pts.iter().enumerate() {
        let p = pt(*px);
        let selected = sel == Some(i);
        let color = if selected { CYAN } else { GREEN };
        imgproc::circle(panel, p, 6, color, 2, imgproc::LINE_AA, 0)?;
        if selected {
            // 이중 링 + 채운 중심 — 어느 점을 조정 중인지 한눈에.
            imgproc::circle(panel, p, 10, CYAN, 1, imgproc::LINE_AA, 0)?;
            imgproc::circle(panel, p, 2, CYAN, -1, imgproc::LINE_AA, 0)?;
        }
        let label = format!("{}:{}", i + 1, marks[i].id);
        imgproc::put_text(
            panel,
            &label,
            Point::new(p.x + 8, p.y - 8),
            imgproc::FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            imgproc::LINE_AA,
            false,
        )?;
    }
    return Ok(());
}

/// 선택된 점의 anchor(회색 x)와 `r` refine이 움직일 수 있는 반경 원.
pub fn draw_anchor_bound(
    panel: &mut Mat,
    anchor: &[camera::Pixel],
    pad: i32,
    sel: Option<usize>,
    radius: f64,
) -> Result<()> {
    let Some(i) = sel else {
        return Ok(());
    };
    let Some(a) = anchor.get(i) else {
        return Ok(());
    };
    let p = pt(to_canvas(*a, pad));
    imgproc::draw_marker(
        panel,
        p,
        GRAY,
        imgproc::MARKER_TILTED_CROSS,
        8,
        1,
        imgproc::LINE_AA,
    )?;
    if radius >= 1.0 {
        imgproc::circle(
            panel,
            p,
            radius.round() as i32,
            GRAY,
            1,
            imgproc::LINE_AA,
            0,
        )?;
    }
    return Ok(());
}

/// PnP 해의 이상 재투영(마젠타 메시·x) + 클릭↔이상 잔차(노랑).
pub fn draw_reproj_overlay(
    panel: &mut Mat,
    clicks: &[camera::Pixel],
    marks: &[Landmark],
    params: &camera::Params,
    pad: i32,
    sel: Option<usize>,
) -> Result<()> {
    if clicks.len() != TABLE_LANDMARK_COUNT {
        return Ok(());
    }
    let ideals: Vec<Option<camera::Pixel>> = marks
        .iter()
        .map(|m| {
            params
                .project_world_unclipped(m.world)
                .map(|p| to_canvas(p, pad))
        })
        .collect();
    let click_pts = to_canvas_pts(clicks, pad);
    let Some(ideal_pts): Option<Vec<camera::Pixel>> = ideals.iter().cloned().collect() else {
        draw_residuals_partial(panel, &click_pts, &ideals, sel)?;
        return Ok(());
    };

    draw_mesh_edges(panel, &ideal_pts, MAGENTA, 2)?;
    draw_residuals_partial(panel, &click_pts, &ideals, sel)?;
    return Ok(());
}

/// 선택된 점은 잔차가 작아도 수치를 항상 보여준다 (조정 중 피드백).
fn draw_residuals_partial(
    panel: &mut Mat,
    clicks: &[camera::Pixel],
    ideals: &[Option<camera::Pixel>],
    sel: Option<usize>,
) -> Result<()> {
    for (i, click) in clicks.iter().enumerate() {
        let Some(ideal) = ideals[i] else {
            continue;
        };
        let c = pt(*click);
        let p = pt(ideal);
        imgproc::line(panel, c, p, YELLOW, 1, imgproc::LINE_AA, 0)?;
        imgproc::draw_marker(
            panel,
            p,
            MAGENTA,
            imgproc::MARKER_CROSS,
            14,
            2,
            imgproc::LINE_AA,
        )?;
        let du = click.x - ideal.x;
        let dv = click.y - ideal.y;
        let err = (du * du + dv * dv).sqrt();
        if err >= 1.5 || sel == Some(i) {
            imgproc::put_text(
                panel,
                &format!("{err:.1}"),
                Point::new(p.x + 8, p.y + 14),
                imgproc::FONT_HERSHEY_SIMPLEX,
                0.4,
                YELLOW,
                1,
                imgproc::LINE_AA,
                false,
            )?;
        }
    }
    return Ok(());
}

/// 클릭↔이상 재투영 거리 [px]. 카메라 뒤로 가면 `None`.
pub fn per_point_residuals(clicks: &[camera::Pixel], params: &camera::Params) -> Vec<Option<f64>> {
    let marks = TablePnp::landmarks();
    return clicks
        .iter()
        .enumerate()
        .map(|(i, click)| {
            let ideal = params.project_world_unclipped(marks[i].world)?;
            let du = click.x - ideal.x;
            let dv = click.y - ideal.y;
            return Some((du * du + dv * dv).sqrt());
        })
        .collect();
}
