//! 조립된 본선 검출기 — mask / roi / scorer 스냅샷.

use anyhow::Result;

use crate::camera;
use crate::camera::Frame;
use crate::detector::spatial::FloorEdgeMask;
use crate::detector::{RoiTrack, ScorerParams};

use super::builder::DetectorBuilder;

pub struct Detector {
    pub mask: FloorEdgeMask,
    pub roi: RoiTrack,
    /// 면적 밴드 HUD용 스냅샷.
    pub scorer: ScorerParams,
}

impl Detector {
    pub fn builder() -> DetectorBuilder {
        return DetectorBuilder::default();
    }

    /// sim: 카메라가 이미 넣은 힌트 픽셀을 그대로 쓴다.
    pub fn passthrough(hint: Option<camera::Pixel>) -> Option<camera::Pixel> {
        return super::passthrough_detect(hint);
    }

    /// 렌즈 왜곡 보정. 실패 시 에러 문자열.
    pub fn undistort(frame: &Frame, params: &crate::camera::Params) -> Result<Frame, String> {
        return super::undistort_frame(frame, params);
    }

    pub fn set_roi_enabled(&mut self, enabled: bool) {
        self.roi.set_roi_enabled(enabled);
    }

    pub fn detect(&mut self, frame: &Frame) -> Option<camera::Pixel> {
        let Ok(masked) = self.mask.apply_bgr(&frame.image) else {
            return None;
        };
        let gated = Frame {
            camera_id: frame.camera_id,
            image: masked,
            timestamp: frame.timestamp,
        };
        return self.roi.detect(&gated);
    }

    pub fn last_area(&self) -> Option<f64> {
        return self.roi.last_area();
    }
}

impl std::fmt::Display for Detector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(f, "detector({})", self.roi);
    }
}
