//! 월드 점들을 이어 그린다 — 궤적 오버레이.

use opencv::Result as CvResult;
use opencv::core::{Mat, Point, Scalar};
use opencv::imgproc;

use crate::Point3;
use crate::camera;

/// 좌표 폭주 방지 — OpenCV에 넘기기 전 자른다.
const CLAMP_PX: f64 = 20_000.0;
/// 밑선 — 배경이 밝든 어둡든 경계가 서게.
const OUTLINE: Scalar = Scalar::new(0.0, 0.0, 0.0, 0.0);
const OUTLINE_EXTRA_PX: i32 = 2;

/// 월드 궤적을 이 카메라로 재투영해 이어 그린다.
///
/// 프레임 밖 좌표도 그대로 그린다 (`project_world_unclipped`) — 잘라 버리면 "궤적이
/// 없는 것"과 "화각을 벗어난 것"이 구분되지 않는다. 카메라 뒤로 넘어간 점에서만 끊는다.
pub fn draw_world_track(
    img: &mut Mat,
    params: &camera::Params,
    points: &[Point3],
    color: Scalar,
    thickness: i32,
) -> CvResult<()> {
    let segments = project_all(params, points);
    // 밑선을 다 깔고 나서 색을 얹는다 — 선끼리 겹쳐도 밑선이 위를 덮지 않게.
    for (from, to) in &segments {
        imgproc::line(
            img,
            *from,
            *to,
            OUTLINE,
            thickness + OUTLINE_EXTRA_PX,
            imgproc::LINE_AA,
            0,
        )?;
    }
    for (from, to) in &segments {
        imgproc::line(img, *from, *to, color, thickness, imgproc::LINE_AA, 0)?;
    }
    return Ok(());
}

fn project_all(params: &camera::Params, points: &[Point3]) -> Vec<(Point, Point)> {
    let mut segments = Vec::new();
    let mut previous: Option<Point> = None;
    for point in points {
        let Some(pixel) = params.project_world_unclipped(*point) else {
            previous = None;
            continue;
        };
        let current = Point::new(
            pixel.x.clamp(-CLAMP_PX, CLAMP_PX) as i32,
            pixel.y.clamp(-CLAMP_PX, CLAMP_PX) as i32,
        );
        if let Some(prev) = previous {
            segments.push((prev, current));
        }
        previous = Some(current);
    }
    return segments;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::Calibration;

    /// 카메라 뒤로 넘어간 점에서 선이 끊겨야 한다 — 이어 그리면 화면을 가로지르는
    /// 가짜 선이 생긴다.
    #[test]
    fn a_point_behind_the_camera_breaks_the_line() {
        let params = Calibration::sim(2).cameras[0].clone();
        let eye = -params.rotation.transpose() * params.translation;
        let front = Point3::new(0.7, 1.3, 0.9);
        // 카메라를 지나 뒤쪽으로 간 점.
        let behind = Point3::from(eye + (eye - front.coords));
        assert_eq!(project_all(&params, &[front, front]).len(), 1);
        assert!(
            project_all(&params, &[front, behind, front]).is_empty(),
            "뒤 점을 사이에 두면 선분이 없어야 한다"
        );
    }
}
