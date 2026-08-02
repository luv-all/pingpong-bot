//! 색 상자 레이어. `tune-colormask`가 만든 축 정렬 범위로 자른다.

use anyhow::Result;
use opencv::core::{Mat, Point, Scalar, Vec3b};
use opencv::prelude::*;
use opencv::{core, imgproc};

use crate::camera::{self, Frame};
use crate::defaults::colormask_for;
use crate::detector::{ColorSpace, ColormaskParams};

use super::super::{Layer, Mask};

/// `inRange` 상자. 양성 표본만으로 만들어지므로 배경이 같은 상자에 들어오면 못 가른다.
///
/// [`super::ColorPlane`]과 같은 자리에 꽂아 비교하려고 둔다. 제외 표본이 모이기 전까지는
/// 이쪽이 유일하게 쓸 수 있는 색 레이어다.
pub struct ColorBox {
    params: ColormaskParams,
    /// 켜진 픽셀만 모아 한 번에 변환한다.
    on: Mat,
    packed: Mat,
    converted: Mat,
    verdict: Mat,
}

impl ColorBox {
    pub fn new(params: ColormaskParams) -> Self {
        return Self {
            params,
            on: Mat::default(),
            packed: Mat::default(),
            converted: Mat::default(),
            verdict: Mat::default(),
        };
    }

    /// `data/colormask.json`에서.
    pub fn load(camera_id: camera::Id) -> Result<Self> {
        return Ok(Self::new(colormask_for(camera_id)?));
    }

    fn code(&self) -> i32 {
        return match self.params.space {
            ColorSpace::Hsv => imgproc::COLOR_BGR2HSV,
            ColorSpace::Ycrcb => imgproc::COLOR_BGR2YCrCb,
        };
    }
}

impl Layer for ColorBox {
    fn name(&self) -> &'static str {
        return "colorbox";
    }

    fn narrow(&mut self, frame: &Frame, mask: &mut Mask) -> Result<()> {
        core::find_non_zero(&*mask, &mut self.on)?;
        let count = self.on.rows();
        if count == 0 {
            return Ok(());
        }

        // 켜진 픽셀을 1×N 으로 모아 cvtColor 를 한 번만 부른다.
        self.packed = Mat::new_rows_cols_with_default(1, count, core::CV_8UC3, Scalar::all(0.0))?;
        for i in 0..count {
            let at: Point = *self.on.at(i)?;
            *self.packed.at_2d_mut::<Vec3b>(0, i)? = *frame.image.at_2d::<Vec3b>(at.y, at.x)?;
        }
        let code = self.code();
        imgproc::cvt_color(
            &self.packed,
            &mut self.converted,
            code,
            0,
            core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )?;

        let p = &self.params;
        core::in_range(
            &self.converted,
            &Scalar::new(p.c0_min.into(), p.c1_min.into(), p.c2_min.into(), 0.0),
            &Scalar::new(p.c0_max.into(), p.c1_max.into(), p.c2_max.into(), 0.0),
            &mut self.verdict,
        )?;

        for i in 0..count {
            if *self.verdict.at_2d::<u8>(0, i)? == 0 {
                let at: Point = *self.on.at(i)?;
                *mask.at_2d_mut::<u8>(at.y, at.x)? = 0;
            }
        }
        return Ok(());
    }
}
