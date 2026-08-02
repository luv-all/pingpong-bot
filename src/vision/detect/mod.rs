//! 프레임 하나에서 공 하나를 찾는다. 색은 판별자가 아니라 후보 생성기다.

pub mod colormask;
mod layer;
pub mod layers;
mod pick;

pub use colormask::{ColorSpace, ColormaskParams};
pub use layer::Layer;
pub use layers::{Background, ColorBox, ColorPlane, Volume, background};
pub use pick::{MIN_CIRCULARITY, Picker};

use anyhow::Result;
use opencv::core::{Mat, Scalar};
use opencv::prelude::*;

use crate::camera::{self, Frame, Pixel};

/// 아직 공일 수 있는 픽셀 (`CV_8UC1`, 255 = 살아있음). 프레임 크기와 같다.
pub type Mask = Mat;

/// 한 프레임에서 잰 값. 점수는 [`Picker::deviation`]이 낸다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Candidate {
    pub pixel: Pixel,
    pub radius_px: f64,
    pub circularity: f64,
}

/// 레이어를 순서대로 돌린 뒤 [`Picker`]로 하나를 고른다.
pub struct Detector {
    /// 순서 = 실행 순서.
    layers: Vec<Box<dyn Layer>>,
    /// 종단. 마스크가 아니라 후보를 내므로 [`Layer`]가 아니다.
    picker: Picker,
    /// 프레임마다 재할당하지 않는다.
    mask: Mask,
}

impl Detector {
    pub fn new(layers: Vec<Box<dyn Layer>>, picker: Picker) -> Self {
        return Self {
            layers,
            picker,
            mask: Mat::default(),
        };
    }

    /// 본선 캐스케이드. 싼 것부터 — 부피는 정적 AND 라 가장 싸다.
    ///
    /// 이 조립이 SSOT 다. 실기 워커도 툴도 여기를 부른다. 레이어를 갈아끼울 땐 여기만
    /// 고치면 되고, 개수가 변해도 `detect-full` 은 그대로 돈다.
    pub fn for_camera(params: &camera::Params) -> Result<Self> {
        let layers: Vec<Box<dyn Layer>> = vec![
            Box::new(Volume::from_calib(params)?),
            Box::new(Background::new(
                background::HISTORY,
                background::VAR_THRESHOLD,
                background::SCALE,
                background::LEARNING_RATE,
            )?),
            Box::new(ColorBox::load(params.camera_id)?),
        ];
        return Ok(Self::new(
            layers,
            Picker::from_calib(params, MIN_CIRCULARITY)?,
        ));
    }

    /// `expect`: 트랙이 있으면 그 깊이의 기대 반지름 [px].
    pub fn detect(&mut self, frame: &Frame, expect: Option<f64>) -> Result<Option<Candidate>> {
        self.reset_mask(frame)?;
        for layer in &mut self.layers {
            layer.narrow(frame, &mut self.mask)?;
        }
        return self.picker.pick(&self.mask, expect);
    }

    /// 후보 전부와 편차. 몇 개가 남는지가 연관이 필요한지의 근거다.
    pub fn candidates(
        &mut self,
        frame: &Frame,
        expect: Option<f64>,
    ) -> Result<Vec<(Candidate, f64)>> {
        self.reset_mask(frame)?;
        for layer in &mut self.layers {
            layer.narrow(frame, &mut self.mask)?;
        }
        return self.picker.candidates(&self.mask, expect);
    }

    /// 단계별 마스크를 남기며 돈다. 툴 전용이라 마스크를 복사한다.
    pub fn trace(
        &mut self,
        frame: &Frame,
        expect: Option<f64>,
    ) -> Result<(Vec<(&'static str, Mask)>, Vec<(Candidate, f64)>)> {
        self.reset_mask(frame)?;
        let mut stages = Vec::with_capacity(self.layers.len());
        for layer in &mut self.layers {
            layer.narrow(frame, &mut self.mask)?;
            stages.push((layer.name(), self.mask.try_clone()?));
        }
        let candidates = self.picker.candidates(&self.mask, expect)?;
        return Ok((stages, candidates));
    }

    /// 전부 켠 상태로 되돌린다.
    fn reset_mask(&mut self, frame: &Frame) -> Result<()> {
        let (rows, cols) = (frame.image.rows(), frame.image.cols());
        if self.mask.rows() != rows || self.mask.cols() != cols {
            self.mask = Mat::new_rows_cols_with_default(
                rows,
                cols,
                opencv::core::CV_8UC1,
                Scalar::all(255.0),
            )?;
            return Ok(());
        }
        self.mask
            .set_to(&Scalar::all(255.0), &opencv::core::no_array())?;
        return Ok(());
    }
}
