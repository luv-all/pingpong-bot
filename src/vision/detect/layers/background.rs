//! 정지한 것을 끈다 — OpenCV MOG2.

use anyhow::Result;
use opencv::core::{Mat, Ptr, Size};
use opencv::prelude::*;
use opencv::{core, imgproc, video};

use crate::camera::Frame;

use super::super::{Layer, Mask};

/// 배경 모델이 기억하는 프레임 수. 정상 상태 학습률은 `1/HISTORY`다.
pub const HISTORY: i32 = 500;
/// 마할라노비스 제곱 임계 — OpenCV 기본값.
pub const VAR_THRESHOLD: f64 = 16.0;
/// 모델을 돌릴 배율. 공 반지름이 3~18 px라 절반에서도 남는다.
pub const SCALE: f64 = 0.5;
/// 음수는 자동. MOG2 구현은 `1 / min(2·nframes, HISTORY)`를 쓴다. 초반이 커서 모델이
/// 빨리 서고, `2·nframes ≥ HISTORY`부터 `1/HISTORY`로 고정된다.
pub const LEARNING_RATE: f64 = -1.0;

/// 배경 차분 레이어. MOG2로 정지한 픽셀을 끈다.
///
/// fly_04 실측 전경 비율 0.04 % (`tests/diag_background.rs`).
pub struct Background {
    mog2: Ptr<video::BackgroundSubtractorMOG2>,
    scale: f64,
    learning_rate: f64,
    small: Mat,
    fg_small: Mat,
    fg: Mat,
    scratch: Mat,
}

impl Background {
    /// 그림자 검출은 끈다. 켜면 마스크가 0/127/255 삼진이 된다.
    pub fn new(history: i32, var_threshold: f64, scale: f64, learning_rate: f64) -> Result<Self> {
        return Ok(Self {
            mog2: video::create_background_subtractor_mog2(history, var_threshold, false)?,
            scale,
            learning_rate,
            small: Mat::default(),
            fg_small: Mat::default(),
            fg: Mat::default(),
            scratch: Mat::default(),
        });
    }
}

impl Layer for Background {
    fn name(&self) -> &'static str {
        return "background";
    }

    fn narrow(&mut self, frame: &Frame, mask: &mut Mask) -> Result<()> {
        let full = mask.size()?;

        let source = if self.scale < 1.0 {
            imgproc::resize(
                &frame.image,
                &mut self.small,
                Size::default(),
                self.scale,
                self.scale,
                imgproc::INTER_AREA,
            )?;
            &self.small
        } else {
            &frame.image
        };

        // 양쪽 트레잇에 `apply`가 있어 모호하다.
        video::BackgroundSubtractorTrait::apply(
            &mut self.mog2,
            source,
            &mut self.fg_small,
            self.learning_rate,
        )?;

        // INTER_NEAREST. 보간하면 0/255 사이 중간값이 생긴다.
        let foreground = if self.scale < 1.0 {
            imgproc::resize(
                &self.fg_small,
                &mut self.fg,
                full,
                0.0,
                0.0,
                imgproc::INTER_NEAREST,
            )?;
            &self.fg
        } else {
            &self.fg_small
        };

        core::bitwise_and(&*mask, foreground, &mut self.scratch, &core::no_array())?;
        std::mem::swap(mask, &mut self.scratch);
        return Ok(());
    }
}
