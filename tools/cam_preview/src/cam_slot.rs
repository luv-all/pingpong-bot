//! 프리뷰 슬롯 상태.

use opencv::prelude::*;
use pingpong_bot::camera;
use pingpong_bot::vision::Detector;

use crate::fps_meter::FpsMeter;
use crate::live_source::LiveSource;

pub struct CamSlot {
    pub label: String,
    pub source: LiveSource,
    pub fourcc_label: String,
    pub reported_fps: Option<f64>,
    pub reported_size: Option<(i32, i32)>,
    pub exposure_backend: String,
    pub meter: FpsMeter,
    pub panel: Option<Mat>,
    /// `--track` 일 때만. 없으면 이 카메라는 그냥 보여 주기만 한다.
    pub params: Option<camera::Params>,
    pub detector: Option<Detector>,
    /// 이번 프레임에서 찾은 공.
    pub found: Option<camera::Pixel>,
}
