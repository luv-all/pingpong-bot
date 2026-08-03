//! 카메라 프레임 위에 궤적들을 재투영해 겹친다.
//!
//! 여기가 이 툴의 본론이다 — 실제와 예측이 **픽셀 단위로** 얼마나 벌어지는지가 보인다.
//! 3D 그림보다 정확한데, 검출이 실제로 본 좌표계 위에서 재기 때문이다.
//!
//! 카메라 뒤로 넘어간 점은 투영이 없다 (`project_world_unclipped` → `None`). 그 자리에서
//! 선을 **끊는다** — 이어 버리면 화면을 가로지르는 가짜 선이 생긴다.
//!
//! # 색
//!
//! 흰색·하늘색은 밝은 실내 영상 위에서 죽는다 (벽·천장·테이블이 이미 밝다). 채도가 높고
//! 서로 보색인 **초록 ↔ 자홍**을 쓰고, 공(주황)과도 겹치지 않게 둔다. 모든 선은 검은
//! 밑선을 한 겹 깔고 그 위에 그린다 — 배경이 무엇이든 경계가 선다.

use anyhow::Result;
use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use pingpong_bot::Point3;
use pingpong_bot::camera::{self, Preview};

/// 실제 궤적, 지금까지 — 초록, 굵게.
const ACTUAL_PAST: Scalar = Scalar::new(60.0, 255.0, 60.0, 0.0);
/// 실제 궤적, 아직 안 온 구간 — 같은 색을 죽여서. 오프라인 재생이라 미래를 이미 알지만,
/// 과거와 같은 굵기로 그리면 "지금 아는 것"과 구분이 안 된다.
const ACTUAL_FUTURE: Scalar = Scalar::new(30.0, 110.0, 30.0, 0.0);
/// EKF 가 보정한 궤적 — 하늘색. 초록(생 삼각측량) 바로 옆에 그려져야 필터가 얼마나
/// 흔들리는 입력을 폈는지 보인다.
const FILTERED: Scalar = Scalar::new(255.0, 200.0, 0.0, 0.0);
/// 커밋 순간에 얼린 예측 — 자홍, 굵게. 초록과 보색이라 겹쳐도 갈린다.
const COMMITTED: Scalar = Scalar::new(255.0, 0.0, 255.0, 0.0);
/// 밑선 — 밝은 배경에서도 검출 원이 보이게.
const OUTLINE: Scalar = Scalar::new(0.0, 0.0, 0.0, 0.0);
/// 검출 픽셀 — 노랑. 선들과 다른 채널이라 안 묻힌다.
const DETECTED: Scalar = Scalar::new(0.0, 255.0, 255.0, 0.0);
/// EKF 추정 위치 재투영 — 흰 테두리 작은 원.
const EKF: Scalar = Scalar::new(255.0, 255.0, 255.0, 0.0);

/// 창에 그릴 궤적들.
pub struct Tracks<'a> {
    /// 생 삼각측량, 현재 프레임까지 — 필터를 안 거친 것.
    pub actual_past: &'a [Point3],
    /// 생 삼각측량, 현재 프레임 이후 (pass 1이 이미 알고 있다).
    pub actual_future: &'a [Point3],
    /// EKF 가 보정한 궤적 (`Trajectory::measured`), 현재 프레임까지.
    pub filtered: &'a [Point3],
    /// 커밋 순간에 얼린 예측. 커밋 전이면 비어 있다.
    pub committed: &'a [Point3],
}

/// 프레임 사본에 오버레이를 그려 돌려준다.
pub fn draw(
    frame: &Mat,
    params: &camera::Params,
    tracks: &Tracks<'_>,
    detected: Option<camera::Pixel>,
    ekf: Option<Point3>,
    label: &str,
    hud: &[String],
) -> Result<Mat> {
    let mut img = frame.try_clone()?;

    // 덜 중요한 것부터 — 겹치면 나중에 그린 게 위로 온다.
    Preview::draw_world_track(&mut img, params, tracks.actual_future, ACTUAL_FUTURE, 1)?;
    Preview::draw_world_track(&mut img, params, tracks.actual_past, ACTUAL_PAST, 3)?;
    Preview::draw_world_track(&mut img, params, tracks.filtered, FILTERED, 2)?;
    Preview::draw_world_track(&mut img, params, tracks.committed, COMMITTED, 3)?;

    if let Some(point) = ekf
        && let Some(pixel) = params.project_world_unclipped(point)
    {
        Preview::draw_circle_px(&mut img, pixel, 7, EKF, 1)?;
    }
    if let Some(pixel) = detected {
        Preview::draw_circle_px(&mut img, pixel, 11, OUTLINE, 4)?;
        Preview::draw_circle_px(&mut img, pixel, 11, DETECTED, 2)?;
    }

    Preview::draw_debug_lines(&mut img, hud, EKF)?;
    Preview::draw_cam_label(&mut img, label, EKF)?;
    return Ok(img);
}
