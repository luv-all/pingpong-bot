//! 프리뷰 슬롯 상태.

use opencv::prelude::*;

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
}
