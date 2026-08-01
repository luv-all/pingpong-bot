//! 카메라 프레임 위에 궤적 두 개를 재투영해 겹친다.
//!
//! 여기가 이 툴의 본론이다 — 흰 선(실제)과 하늘색 선(예측)이 **픽셀 단위로** 얼마나
//! 벌어지는지가 보인다. 3D 그림보다 정확한데, 검출이 실제로 본 좌표계 위에서 재기 때문이다.
//!
//! 카메라 뒤로 넘어간 점은 투영이 없다 (`project_world_unclipped` → `None`). 그 자리에서
//! 선을 **끊는다** — 이어 버리면 화면을 가로지르는 가짜 선이 생긴다.

use anyhow::Result;
use opencv::core::{Mat, Point, Scalar};
use opencv::imgproc;
use opencv::prelude::*;
use pingpong_bot::Point3;
use pingpong_bot::camera::{self, Preview};

/// 좌표 폭주 방지 — OpenCV에 넘기기 전 자른다.
const DRAW_CLAMP_PX: f64 = 20_000.0;

const WHITE: Scalar = Scalar::new(255.0, 255.0, 255.0, 0.0);
const CYAN: Scalar = Scalar::new(255.0, 255.0, 0.0, 0.0);
const GREEN: Scalar = Scalar::new(0.0, 255.0, 0.0, 0.0);
const MAGENTA: Scalar = Scalar::new(255.0, 0.0, 255.0, 0.0);

/// 프레임 사본에 오버레이를 그려 돌려준다.
pub fn draw(
    frame: &Mat,
    params: &camera::Params,
    observed: &[Point3],
    predicted: &[Point3],
    detected: Option<camera::Pixel>,
    ekf: Option<Point3>,
    label: &str,
    hud: &[String],
) -> Result<Mat> {
    let mut img = frame.try_clone()?;

    track(&mut img, params, observed, WHITE, 2)?;
    track(&mut img, params, predicted, CYAN, 1)?;

    if let Some(point) = ekf
        && let Some(pixel) = params.project_world_unclipped(point)
    {
        Preview::draw_circle_px(&mut img, pixel, 7, MAGENTA, 1)?;
    }
    if let Some(pixel) = detected {
        Preview::draw_circle_px(&mut img, pixel, 10, GREEN, 2)?;
    }

    Preview::draw_debug_lines(&mut img, hud, WHITE)?;
    Preview::draw_cam_label(&mut img, label, WHITE)?;
    return Ok(img);
}

fn track(
    img: &mut Mat,
    params: &camera::Params,
    points: &[Point3],
    color: Scalar,
    thickness: i32,
) -> Result<()> {
    let mut previous: Option<Point> = None;
    for point in points {
        let Some(pixel) = params.project_world_unclipped(*point) else {
            // 카메라 뒤 — 여기서 끊는다.
            previous = None;
            continue;
        };
        let current = pt(pixel.x, pixel.y);
        if let Some(prev) = previous {
            imgproc::line(img, prev, current, color, thickness, imgproc::LINE_AA, 0)?;
        }
        previous = Some(current);
    }
    return Ok(());
}

fn pt(x: f64, y: f64) -> Point {
    return Point::new(
        x.clamp(-DRAW_CLAMP_PX, DRAW_CLAMP_PX) as i32,
        y.clamp(-DRAW_CLAMP_PX, DRAW_CLAMP_PX) as i32,
    );
}
