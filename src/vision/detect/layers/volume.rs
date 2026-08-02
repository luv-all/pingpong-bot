//! 공간 레이어. 공이 날 수 있는 부피 밖을 끈다.

use anyhow::Result;
use opencv::core::{Mat, Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use crate::Point3;
use crate::camera::{self, Frame};
use crate::constants::table;

use super::super::{Layer, Mask};

/// 테이블 옆으로 벌릴 여유 [m].
pub const SIDE_MARGIN: f64 = 0.3;
/// 로봇 쪽으로 벌릴 여유 [m]. 공을 로봇 위치까지 따라가야 한다.
pub const ROBOT_MARGIN: f64 = 0.5;
/// 상판 아래 여유와 위쪽 비행 높이 [m].
pub const BELOW: f64 = 0.1;
pub const FLIGHT_BAND: f64 = 1.0;

/// 공이 날 수 있는 3D 부피를 이 카메라로 투영해 만든 keep 마스크.
///
/// 부피가 볼록이므로 투영도 볼록이다. 여덟 꼭짓점의 컨벡스 헐을 채우면 끝이고, 카메라가
/// 고정이라 **한 번만 계산한다**. 매 프레임 비용은 AND 한 번이다.
///
/// 꼭짓점이 카메라 뒤로 넘어가 셋 미만이 남으면 전부 통과시킨다. 화면을 통째로 지우는
/// 것보다 낫다.
///
/// 값은 싸지만 효과도 작다 — 실측 keep 이 cam0 85 %, cam1 74 %다. 카메라가 3 m 거리인데
/// 부피가 2.1×3.7×1.1 m 라 투영하면 화면을 거의 채운다.
pub struct Volume {
    keep: Mask,
    /// 프레임마다 재할당하지 않는다.
    scratch: Mat,
}

impl Volume {
    pub fn from_calib(params: &camera::Params) -> Result<Self> {
        let (x0, x1) = (-SIDE_MARGIN, table::WIDTH_X + SIDE_MARGIN);
        let (y0, y1) = (-ROBOT_MARGIN, table::LENGTH_Y + SIDE_MARGIN);
        let (z0, z1) = (table::SURFACE_Z - BELOW, table::SURFACE_Z + FLIGHT_BAND);
        let mut projected = Vector::<Point>::new();
        for x in [x0, x1] {
            for y in [y0, y1] {
                for z in [z0, z1] {
                    if let Some(pixel) = params.project_world_unclipped(Point3::new(x, y, z)) {
                        projected.push(Point::new(pixel.x.round() as i32, pixel.y.round() as i32));
                    }
                }
            }
        }

        let (w, h) = (params.width as i32, params.height as i32);
        let fill = if projected.len() >= 3 { 0.0 } else { 255.0 };
        let mut keep =
            Mat::new_rows_cols_with_default(h, w, opencv::core::CV_8UC1, Scalar::all(fill))?;
        if projected.len() >= 3 {
            let mut hull = Vector::<Point>::new();
            imgproc::convex_hull(&projected, &mut hull, false, true)?;
            imgproc::fill_convex_poly(&mut keep, &hull, Scalar::all(255.0), imgproc::LINE_8, 0)?;
        }
        return Ok(Self {
            keep,
            scratch: Mat::default(),
        });
    }

    /// keep 비율 [%]. 이 레이어가 얼마나 지우는지 보는 값.
    pub fn keep_ratio(&self) -> Result<f64> {
        let on = opencv::core::count_non_zero(&self.keep)?;
        let total = self.keep.rows() * self.keep.cols();
        return Ok(100.0 * f64::from(on) / f64::from(total.max(1)));
    }
}

impl Layer for Volume {
    fn name(&self) -> &'static str {
        return "volume";
    }

    fn narrow(&mut self, _frame: &Frame, mask: &mut Mask) -> Result<()> {
        opencv::core::bitwise_and(
            &*mask,
            &self.keep,
            &mut self.scratch,
            &opencv::core::no_array(),
        )?;
        std::mem::swap(mask, &mut self.scratch);
        return Ok(());
    }
}
